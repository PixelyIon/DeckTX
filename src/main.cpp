/* SPDX-License-Identifier: MPL-2.0
 * Copyright © Mark Collins
 */
#include <functional>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <SDL3/SDL.h>
#include <SDL3/SDL_joystick.h>
#include <SDL3/SDL_main.h>
#include <backends/imgui_impl_sdl3.h>
#include <backends/imgui_impl_sdlrenderer3.h>
#include <imgui.h>

#include "common/scope.h"
#include "common/thread_name.h"

#include "crsf/crsf.h"
#include "crsf/parameter.h"
#include "serial/serial.h"

class ElrsManager {
  private:
    crsf::Interface crsfInterface;

    std::mutex stateMutex;
    std::condition_variable stateCV;

    std::vector<crsf::Device> devices;
    std::jthread rxThread, txThread;
    bool earlyPing{}; //!< Send a ping frame early, this is used when a packet with an unrecognized device is found.
    size_t pingsSinceLastRx{}; //!< Number of pings sent since the last received packet.

    std::optional<crsf::LinkStatisticsData> linkStatistics{};
    std::optional<crsf::RcChannelsFrame> channelFrame{};
    std::optional<crsf::BatterySensorData> batteryData{};

    SDL_Joystick* joystick{};
    struct ButtonState {
        bool pressed{}; //!< Whether the button is currently pressed, this is necessary to track releases for toggling.
        bool toggled{}; //!< Turns true when the button is pressed and false when it is pressed again.
    };
    std::vector<ButtonState> buttonStates;
    bool showJoystickData{}; //!< Whether to show the joystick input data.

    crsf::Device* FindDevice(crsf::Address address) {
        auto it{std::find_if(devices.begin(), devices.end(), [&](const crsf::Device& device) { return device.address == address; })};
        if (it == devices.end())
            return nullptr;
        return &*it;
    }

    void RxThread(std::stop_token stopToken) {
        SetThreadName("ELRS RX");
        while (!stopToken.stop_requested()) {
            try {
                std::optional<crsf::FrameV> frameV{crsfInterface.ReceiveFrame(std::chrono::steady_clock::now() + std::chrono::milliseconds{100})};
                // Note: Timeouts are important so that stop tokens are checked frequently.
                if (!frameV)
                    continue;

                std::scoped_lock lock{stateMutex};
                std::visit([&](auto&& frame) {
                    using T = std::decay_t<decltype(frame)>;

                    if constexpr (std::is_same_v<T, crsf::BatterySensorFrame>) {
                        batteryData = frame.battery;
                    } else if constexpr (std::is_same_v<T, crsf::LinkStatisticsFrame>) {
                        linkStatistics = frame.statistics;
                    } else if constexpr (std::is_same_v<T, crsf::PingFrame>) {
                        fmt::print("Ping frame\n");
                    } else if constexpr (std::is_same_v<T, crsf::DeviceInfoFrame>) {
                        crsf::Device* device{FindDevice(frame.originAddress)};
                        if (device) {
                            device->Update(frame.deviceName, frame.deviceMetadata);
                            fmt::print("Updated device: {}\n", device->name);
                        } else {
                            device = &devices.emplace_back(frame.originAddress, frame.deviceName, frame.deviceMetadata);
                            fmt::print("Added device: {}\n", device->name);
                        }
                    } else if constexpr (std::is_same_v<T, crsf::ParameterSettingsEntryFrame>) {
                        auto device{FindDevice(frame.originAddress)};
                        if (!device) {
                            earlyPing = true;
                            fmt::print("Device not found: 0x{:X}\n", static_cast<int>(frame.originAddress));
                            return;
                        }

                        crsf::Parameter* parameter{device->FindParameter(frame.fieldIndex)};
                        if (parameter) {
                            if (parameter->complete)
                                return;
                            parameter->ProcessChunk(frame.data, frame.chunksRemaining);
                        } else {
                            fmt::print("Parameter not found: {}\n", frame.fieldIndex);
                            return;
                        }
                    } else if constexpr (std::is_same_v<T, crsf::RadioIdFrame>) {
                        auto device{FindDevice(frame.originAddress)};
                        if (!device) {
                            earlyPing = true;
                            fmt::print("Device not found: 0x{:X}\n", static_cast<int>(frame.originAddress));
                            return;
                        }

                        device->syncData = frame.sync;
                    } else {
                        throw Exception("Unknown frame type: 0x{:X}", static_cast<u8>(frame.type));
                    }
                },
                    *frameV);

                pingsSinceLastRx = 0;
                stateCV.notify_all();
            } catch (std::exception& e) {
                fmt::print("RX Error: {}\n", e.what());
                if (!crsfInterface.IsActive()) {
                    fmt::print("Serial port is inactive, stopping RX thread\n");
                    requestStop = true;
                    return;
                }
            }
        }
    }

    void TxThread(std::stop_token stopToken) {
        SetThreadName("ELRS TX");

        std::chrono::time_point<std::chrono::steady_clock> wakeTime{std::chrono::steady_clock::now()}, pingTime{wakeTime};
        constexpr std::chrono::milliseconds WakeInterval{100}, PingInterval{5000};

        bool recurringException{};
        while (!stopToken.stop_requested()) {
            std::unique_lock lock{stateMutex};
            stateCV.wait_until(lock, wakeTime > pingTime ? wakeTime : pingTime);

            try {
                if (pingsSinceLastRx >= 2) {
                    fmt::print("No packets received after two pings, resetting device\n");
                    crsfInterface.ResetDevice();
                    pingsSinceLastRx = 0;
                    earlyPing = true;
                }

                for (crsf::Device& device : devices) {
                    auto frame{device.TryGetTxFrame()};

                    if (frame) {
                        crsfInterface.SendFrame(*frame);
                    } else if (device.address == crsf::Address::FlightController && channelFrame) {
                        crsfInterface.SendFrame(*channelFrame);
                        channelFrame.reset();
                    }
                }

                if (earlyPing || std::chrono::steady_clock::now() >= pingTime) {
                    crsf::PingFrame ping{crsf::Address::Broadcast, crsf::Address::ElrsLua};
                    crsfInterface.SendFrame(ping);
                    fmt::print("Sent ping frame\n");

                    earlyPing = false;
                    pingsSinceLastRx++;
                    pingTime = std::chrono::steady_clock::now() + PingInterval;
                }

                wakeTime = std::chrono::steady_clock::now() + WakeInterval;
            } catch (std::exception& e) {
                fmt::print("TX Error: {}\n", e.what());
                if (!crsfInterface.IsActive()) {
                    fmt::print("Serial port is inactive, stopping TX thread\n");
                    requestStop = true;
                    return;
                }
                if (recurringException) {
                    fmt::print("Recurring exception, trying to reset the device\n");
                    crsfInterface.ResetDevice();
                }
                recurringException = true;
            }
        }
    }

  public:
    bool requestStop{}; //!< If any of the threads have requested a stop, this happens when they can no longer continue due to an error or the serial port being closed.

    ElrsManager(std::string portName, size_t baudRate)
        : crsfInterface{portName, baudRate}
        , rxThread{std::bind_front(&ElrsManager::RxThread, this)}
        , txThread{std::bind_front(&ElrsManager::TxThread, this)} {}

    bool HasJoystick() {
        if (joystick) {
            if (SDL_JoystickConnected(joystick))
                return true;
            SDL_CloseJoystick(joystick);
            joystick = nullptr;
        }

        if (SDL_HasJoystick()) {
            int joystickCount{};
            auto joysticks{SDL_GetJoysticks(&joystickCount)};
            if (joystickCount > 0) {
                joystick = SDL_OpenJoystick(joysticks[0]);
                if (joystick != nullptr) {
                    fmt::print("Opened joystick: {}\n", SDL_GetJoystickName(joystick));
                    buttonStates.resize(SDL_GetNumJoystickButtons(joystick));
                    return true;
                } else {
                    fmt::print("Failed to open joystick: {}\n", SDL_GetError());
                }
            }
        }

        return false;
    }

    void Draw() {
        std::scoped_lock lock{stateMutex};

        try {
            bool wrotePackets{};

            if (linkStatistics) {
                ImGui::Text("Link Statistics:");
                ImGui::Indent();

                ImGui::Text("RF Mode: %s", linkStatistics->RfMode());
                ImGui::Text("Diversity active antenna: %d", linkStatistics->activeAntenna);

                // Uplink
                ImGui::Text("Uplink TX Power: %d mW", linkStatistics->UplinkTxPower());
                ImGui::ProgressBar(linkStatistics->UplinkRssi1Norm(), ImVec2(0.0f, 0.0f), fmt::format("Uplink RSSI Ant. 1: {} dBm", linkStatistics->UplinkRssi1()).c_str());
                ImGui::ProgressBar(linkStatistics->UplinkRssi2Norm(), ImVec2(0.0f, 0.0f), fmt::format("Uplink RSSI Ant. 1: {} dBm", linkStatistics->UplinkRssi2()).c_str());
                ImGui::ProgressBar(linkStatistics->UplinkLinkQualityNorm(), ImVec2(0.0f, 0.0f), fmt::format("Uplink Package success rate: {}%", linkStatistics->uplinkLinkQuality).c_str());
                ImGui::ProgressBar(linkStatistics->UplinkSnrNorm(), ImVec2(0.0f, 0.0f), fmt::format("Uplink SNR: {} dB", linkStatistics->uplinkSnr).c_str());

                // Downlink
                ImGui::ProgressBar(linkStatistics->DownlinkRssiNorm(), ImVec2(0.0f, 0.0f), fmt::format("Downlink RSSI: {} dBm", linkStatistics->DownlinkRssi()).c_str());
                ImGui::ProgressBar(linkStatistics->DownlinkLinkQualityNorm(), ImVec2(0.0f, 0.0f), fmt::format("Downlink package success rate: {}%", linkStatistics->downlinkLinkQuality).c_str());
                ImGui::ProgressBar(linkStatistics->DownlinkSnrNorm(), ImVec2(0.0f, 0.0f), fmt::format("Downlink SNR: {} dB", linkStatistics->downlinkSnr).c_str());

                ImGui::Unindent();

                for (crsf::Device& device : devices) {
                    if (device.syncData) {
                        ImGui::Text("Device: %s (Address: 0x%02hhX)", device.name.c_str(), static_cast<u8>(device.address));
                        ImGui::Indent();
                        ImGui::Text("Packet Rate: %d", device.syncData->packetRate);
                        ImGui::Text("Packet Offset: %d", device.syncData->packetOffset);
                        ImGui::Unindent();
                    }
                }

                ImGui::Separator();
            }

            {
                ImGui::Text("RC Channels:");
                ImGui::Indent();

                if (HasJoystick()) {
                    SDL_LockJoysticks();

                    for (int i{}; i < SDL_GetNumJoystickButtons(joystick); i++) {
                        bool pressed{static_cast<bool>(SDL_GetJoystickButton(joystick, i))};
                        ButtonState& buttonState{buttonStates[i]};
                        if (pressed && !buttonState.pressed) {
                            buttonState.pressed = true;
                            buttonState.toggled = !buttonState.toggled;
                        } else if (!pressed && buttonState.pressed) {
                            buttonState.pressed = false;
                        }
                    }

                    constexpr u16 CrsfChannelMin{172 + 5}, CrsfChannelMid{992}, CrsfChannelMax{1811 - 5}, CrsfChannelDiff{CrsfChannelMax - CrsfChannelMin};
                    constexpr u16 JoystickDeadzone{2000};

                    auto getAxisValue{[&](int axis, bool invert = false) -> u16 {
                        i16 inputValue{SDL_GetJoystickAxis(joystick, axis)};
                        if (invert) {
                            if (inputValue == INT16_MIN)
                                inputValue = INT16_MAX;
                            else
                                inputValue *= -1;
                        }
                        if (abs(inputValue) < JoystickDeadzone)
                            return CrsfChannelMid;

                        // Shift the input value from -32768 to 32767 (signed 16-bit) to 0 to 65535 (unsigned 16-bit)
                        u32 unsignedValue{static_cast<u32>(inputValue) + (INT16_MAX + 1)};

                        // Scale the shifted value to the CRSF range
                        u32 crsfValue{((unsignedValue * CrsfChannelDiff) / UINT16_MAX) + CrsfChannelMin};

                        return static_cast<uint16_t>(crsfValue);
                    }};

                    /* getCombinedTriggerValue, combines anegative axis and the positive axis them into a single axis which is:
                     * * Centered when both are at their minimum. 
                     * * At the maximum value, if the positive axis is at its max.
                     * * At the minimum value, if the negative axis is at its max.
                     */
                    auto getCombinedTriggerValue{[&](int negativeAxis, int positiveAxis, bool invert = false) -> u16 {
                        i16 negativeInputValue{SDL_GetJoystickAxis(joystick, negativeAxis)};
                        if (abs(negativeInputValue) < JoystickDeadzone)
                            negativeInputValue = 0;
                        u32 negativeUnsignedValue{static_cast<u32>(negativeInputValue) + (INT16_MAX + 1)};

                        i16 positiveInputValue{SDL_GetJoystickAxis(joystick, positiveAxis)};
                        if (abs(positiveInputValue) < JoystickDeadzone)
                            positiveInputValue = 0;
                        u32 positiveUnsignedValue{static_cast<u32>(positiveInputValue) + (INT16_MAX + 1)};

                        // Shift the input value from -32768 to 32767 (signed 16-bit) to 0 to 65535 (unsigned 16-bit)
                        u32 unsignedValue{INT16_MAX};
                        unsignedValue += positiveUnsignedValue / 2;
                        unsignedValue -= negativeUnsignedValue / 2;

                        // Scale the shifted value to the CRSF range
                        u32 crsfValue{((unsignedValue * CrsfChannelDiff) / UINT16_MAX) + CrsfChannelMin};

                        return static_cast<uint16_t>(crsfValue);
                    }};

                    auto getButtonToggle{[&](int button) -> u16 {
                        return buttonStates[button].toggled ? CrsfChannelMid : CrsfChannelMin;
                    }};

                    crsf::RcChannelsData crsfChannels{};
                    crsfChannels.set(0, getAxisValue(SDL_GAMEPAD_AXIS_RIGHTX));
                    crsfChannels.set(1, getAxisValue(SDL_GAMEPAD_AXIS_RIGHTY, true));
                    crsfChannels.set(2, getCombinedTriggerValue(SDL_GAMEPAD_AXIS_LEFT_TRIGGER, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER));
                    crsfChannels.set(3, getAxisValue(SDL_GAMEPAD_AXIS_LEFTX));
                    // CRSF Aux 1 value is ignored for some reason.
                    crsfChannels.set(5, getButtonToggle(SDL_GAMEPAD_BUTTON_START));
                    crsfChannels.set(6, getButtonToggle(SDL_GAMEPAD_BUTTON_LEFT_STICK));

                    constexpr std::array<const char*, 7> channelLabels{"Roll", "Pitch", "Throttle", "Yaw", "Aux 1", "Aux 2", "Aux 3"};
                    for (int i{}; i < channelLabels.size(); i++) {
                        float scaledValue{static_cast<float>(crsfChannels.get(i) - CrsfChannelMin) / CrsfChannelDiff};
                        ImGui::ProgressBar(scaledValue, ImVec2(0.0f, 0.0f), channelLabels[i]);
                    }

                    if (!channelFrame || channelFrame->channels != crsfChannels) {
                        channelFrame = crsf::RcChannelsFrame(crsfChannels, crsf::Address::FlightController);
                        wrotePackets = true;
                    }

                    ImGui::Checkbox("Show Joystick Input", &showJoystickData);
                    if (showJoystickData) {
                        ImGui::Text("Name: %s", SDL_GetJoystickName(joystick));
                        for (int i{}; i < SDL_GetNumJoystickAxes(joystick); i++) {
                            ImGui::Text("Axis %d:", i);
                            ImGui::Indent();
                            i16 signedValue{SDL_GetJoystickAxis(joystick, i)};
                            u32 unsignedValue{static_cast<u32>(i32{signedValue} - (std::numeric_limits<i16>::min)())};
                            float value{static_cast<float>(unsignedValue) / (std::numeric_limits<u16>::max)()};
                            ImGui::ProgressBar(value, ImVec2(-100, 0), fmt::format("Axis {}", i).c_str());
                            ImGui::Unindent();
                        }

                        for (int i{}; i < SDL_GetNumJoystickButtons(joystick); i++) {
                            ButtonState& buttonState{buttonStates[i]};
                            ImGui::Text("Button %d:", i);
                            ImGui::Indent();
                            ImGui::Checkbox("Pressed", &buttonState.pressed);
                            ImGui::Checkbox("Toggled", &buttonState.toggled);
                            ImGui::Unindent();
                        }
                    }

                    SDL_UnlockJoysticks();
                } else {
                    ImGui::Text("No joystick connected");
                }

                ImGui::Unindent();
                ImGui::Separator();
            }

            if (batteryData) {
                ImGui::Text("Battery:");
                ImGui::Indent();
                ImGui::Text("Voltage: %.2f V", batteryData->Voltage());
                ImGui::Text("Current: %.2f A", batteryData->Current());
                ImGui::Text("Capacity: %.2f Ah", batteryData->Capacity());
                ImGui::Text("Remaining Capacity: %d%%", batteryData->RemainingCapacity());
                ImGui::Unindent();
                ImGui::Separator();
            }

            for (crsf::Device& device : devices) {
                ImGui::Text("Device: %s (Address: 0x%02hhX, Serial No: 0x%08X)", device.name.c_str(), static_cast<u8>(device.address), device.metadata.serialNo);
                ImGui::Indent();
                ImGui::Text("HW Version: 0x%02X, SW Version: 0x%02X", device.metadata.hardwareVersion, device.metadata.softwareVersion);
                ImGui::Text("Parameters: (v0x%02X - %d)", device.metadata.parameterVersion, device.metadata.fieldCount);

                ImGui::Indent();
                for (crsf::Parameter& parameter : device.parameters) {
                    if (parameter.complete || parameter.activeCommand) {
                        std::visit([&](auto value) {
                            if constexpr (std::is_same_v<decltype(value), crsf::IntegerParameter<u8>> || std::is_same_v<decltype(value), crsf::IntegerParameter<i8>> || std::is_same_v<decltype(value), crsf::IntegerParameter<u16>> || std::is_same_v<decltype(value), crsf::IntegerParameter<u32>>) {
                                i64 integerValue{static_cast<i64>(value.value)};
                                i64 step{1};
                                if (ImGui::InputScalar(parameter.name.data(), ImGuiDataType_U64, &integerValue, &step)) {
                                    if (integerValue < value.min)
                                        integerValue = value.min;
                                    else if (integerValue > value.max)
                                        integerValue = value.max;
                                    value.value = static_cast<decltype(value.value)>(integerValue);
                                    fmt::print("Set '{}' to {}\n", parameter.name, value.value);
                                    device.WriteParameter(parameter.fieldIndex, value.value);
                                    wrotePackets = true;
                                }
                            } else if constexpr (std::is_same_v<decltype(value), crsf::FloatParameter>) {
                                std::string format{"%." + std::to_string(value.precision) + "f"};
                                if (ImGui::InputFloat(parameter.name.data(), &value.value, value.step, value.step * 10, format.c_str())) {
                                    if (value.value < value.min)
                                        value.value = value.min;
                                    else if (value.value > value.max)
                                        value.value = value.max;
                                    fmt::print("Set '{}' to {}\n", parameter.name, value.value);
                                    device.WriteParameter(parameter.fieldIndex, value.value);
                                    wrotePackets = true;
                                }
                            } else if constexpr (std::is_same_v<decltype(value), crsf::TextSelectionParameter>) {
                                ImGui::Text("%s", parameter.name.data());
                                if (ImGui::BeginCombo(fmt::format("##PARAM{}-{}", parameter.fieldIndex, device.name).c_str(), std::string{value.options[value.selectedIndex]}.c_str())) {
                                    for (u8 i{0}; i < value.options.size(); ++i) {
                                        bool selected{i == value.selectedIndex};
                                        if (ImGui::Selectable(fmt::format("{}{}", value.options[i], i == value.defaultIndex ? " (Default)" : "").c_str(), selected)) {
                                            value.selectedIndex = i;
                                            fmt::print("Selected '{}' ({}) in field #{} ({})\n", value.options[i], i, parameter.fieldIndex, parameter.name);
                                            device.WriteParameter(parameter.fieldIndex, static_cast<u8>(i));
                                            wrotePackets = true;
                                        }
                                        if (selected)
                                            ImGui::SetItemDefaultFocus();
                                    }
                                    ImGui::EndCombo();
                                }
                            } else if constexpr (std::is_same_v<decltype(value), crsf::FolderParameter>) {
                                ImGui::Unindent();
                                ImGui::Text("%s", value.displayName.empty() ? parameter.name.data() : value.displayName.data());
                                ImGui::Indent();
                            } else if constexpr (std::is_same_v<decltype(value), crsf::InfoParameter>) {
                                ImGui::Text("%s: %s", parameter.name.data(), value.display.data());
                            } else if constexpr (std::is_same_v<decltype(value), crsf::CommandParameter>) {
                                if (ImGui::Button(parameter.name.data())) {
                                    device.WriteParameter(parameter.fieldIndex, static_cast<u8>(crsf::CommandStep::Click));
                                    wrotePackets = true;
                                } else if (parameter.complete && parameter.activeCommand && parameter.updateTime + std::chrono::seconds{1} < std::chrono::steady_clock::now()) {
                                    device.WriteParameter(parameter.fieldIndex, static_cast<u8>(crsf::CommandStep::Query));
                                    wrotePackets = true;
                                }
                                if (value.step != crsf::CommandStep::Idle) {
                                    ImGui::SameLine();
                                    ImGui::Text("%s", crsf::CommandStepToString(value.step));
                                }
                            } else {
                                ImGui::Text("Unhandled '%s' type: %d", parameter.name.data(), static_cast<int>(parameter.type));
                            }
                        },
                            parameter.value);
                    } else {
                        ImGui::Text("Parameter #%d", parameter.fieldIndex);
                    }
                }
                ImGui::Unindent();

                ImGui::Unindent();
                ImGui::Separator();
            }

            if (wrotePackets)
                stateCV.notify_all();
        } catch (std::exception& e) {
            fmt::print("Error: {}\n", e.what());
            ImGui::Text("Error: %s", e.what());
        }
    }
};

int main(int argv, char** args) {
    /* SDL */
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD) != 0)
        throw Exception("SDL initialization failed: {}", SDL_GetError());

    SDL_Window* window{SDL_CreateWindow("DeckTX", 1280, 720, SDL_WINDOW_VULKAN | SDL_WINDOW_HIGH_PIXEL_DENSITY)};
    if (window == nullptr)
        throw Exception("SDL window creation failed: {}", SDL_GetError());

    /* SDL Renderer */
    SDL_Renderer* renderer{SDL_CreateRenderer(window, nullptr)};
    if (renderer == nullptr)
        throw Exception("SDL renderer creation failed: {}", SDL_GetError());
    SDL_SetRenderVSync(renderer, 1);

    SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
    SDL_SetWindowResizable(window, SDL_TRUE);
    SDL_ShowWindow(window);

    /* IMGUI */
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io{ImGui::GetIO()};
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    io.IniFilename = nullptr;

    ImGui::StyleColorsDark();

    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);

    /* Main Loop */
    std::optional<SerialPortInfo> selectedPort{};
    u32 baudRate{921600};
    std::string error{};
    std::optional<ElrsManager> manager;

    bool running{true};
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);

            if (event.type == SDL_EVENT_QUIT)
                running = false;
        }

        ImGui_ImplSDLRenderer3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        const ImGuiViewport* viewport{ImGui::GetMainViewport()};
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);

        {
            ImGui::Begin("DeckTX", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse);
            SCOPE_EXIT(ImGui::End());

            if (manager) {
                manager->Draw();
                if (ImGui::Button("Disconnect") || manager->requestStop)
                    manager.reset();
            } else {
                std::vector<SerialPortInfo> ports{ListSerialPorts()};
                if (!selectedPort && !ports.empty())
                    selectedPort = ports.front();

                if (ImGui::BeginCombo("Serial Port", selectedPort ? selectedPort->friendlyName.c_str() : "")) {
                    for (const SerialPortInfo& it : ports) {
                        bool selected{selectedPort->devicePath == it.devicePath};
                        if (ImGui::Selectable(it.friendlyName.c_str(), selected))
                            selectedPort = it;
                        if (selected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }

                ImGui::InputScalar("Baud Rate", ImGuiDataType_U32, &baudRate);

                if (ImGui::Button("Connect")) {
                    if (selectedPort) {
                        ImGui::Text("Connecting to port: %s", selectedPort->friendlyName.c_str());
                        fmt::print("Connecting to port: {}\n", selectedPort->friendlyName);

                        try {
                            manager.emplace(selectedPort->devicePath, baudRate);
                        } catch (std::exception& e) {
                            error = e.what();
                            manager.reset();
                        }
                    }
                }

                if (!error.empty())
                    ImGui::Text("Error: %s", error.c_str());
            }
        }

        ImGui::Render();
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);

        SDL_RenderPresent(renderer);
    }

    SDL_DestroyWindow(window);

    SDL_Quit();

    return 0;
}