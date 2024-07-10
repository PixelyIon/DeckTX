/* SPDX-License-Identifier: MPL-2.0
 * Copyright © Mark Collins
 */
#include "protocol.h"
#include <chrono>
#include <thread>

#include "crsf.h"

namespace crsf {
bool Interface::IsActive() {
    return serial.IsOpen();
}

void Interface::ResetDevice() { 
    if (!serial.IsOpen())
        return;

    serial.SetDTR(false);
    serial.SetRTS(false);
    serial.WriteByte(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    serial.SetDTR(true);
    serial.SetRTS(true);
    serial.WriteByte(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void Interface::SendFrame(const FrameV& frame) {
    auto data{[&]() -> FrameData {
        return std::visit([&](auto&& frame) {
            return frame.GetData();
        },
            frame);
    }()};

    // Note: Any frames need to be sent twice to ensure they are received.
    serial.Write(data);
    serial.Write(data);
}

std::optional<FrameV> Interface::ReceiveFrame(std::chrono::time_point<std::chrono::steady_clock> timeout) {
    FrameData frame{};
    bool readAddress{false}; //!< Whether the address has been read while looking for the sync byte.

    while (true) {
        auto optionalByte{serial.ReadByte(timeout)};
        if (!optionalByte)
            return std::nullopt;

        u8 byte{*optionalByte};
        if (byte == SyncByte) {
            break;
        } else if (static_cast<Address>(byte) == Address::CrsfTransmitter
                   || static_cast<Address>(byte) == Address::RadioTransmitter) {
            readAddress = true;
            frame[0] = byte;
            break;
        } else {
            if (byte >= 0x20 && byte <= 0x7E)
                fmt::print("Dropping: '{}'\n", static_cast<char>(byte));
            else
                fmt::print("Dropping: 0x{:02X}\n", static_cast<int>(byte));
        }
    }

    Header* header{reinterpret_cast<Header*>(frame.data_resize(sizeof(Header)))};
    serial.Read(std::span<u8>(frame.data() + readAddress, sizeof(Header) - readAddress));

    if (header->frameSize + FrameSizeNotCountedBytes > FrameSizeMax)
        throw Exception("Frame size too large: 0x{:X}", header->frameSize);
    else if (header->frameSize < FrameTypeSize + FrameCrcSize)
        throw Exception("Frame size too small: 0x{:X}", header->frameSize);

    frame.resize(header->frameSize + FrameSizeNotCountedBytes);
    serial.Read(std::span<u8>(frame.data() + sizeof(Header), header->frameSize - FrameTypeSize));

    u8 crc{CrsfCrc.Calculate(frame.data() + FrameSizeNotCountedBytes, header->frameSize - FrameCrcSize)};
    u8 frameCrc{frame[FrameSizeNotCountedBytes + header->frameSize - FrameCrcSize]};
    if (crc != frameCrc)
        throw Exception("CRC mismatch: expected 0x{:02X}, got 0x{:02X}", crc, frameCrc);

    if (frame == oldFrame)
        return ReceiveFrame(timeout);
    else
        oldFrame = frame;

    return GetFrame(std::span<u8>(frame.data(), header->frameSize + FrameSizeNotCountedBytes), header->type);
}

Interface::Interface(std::string port, size_t baud)
    : serial(port, baud) {}

Device::Device(Address address, const std::string& name, DeviceMetadata metadata)
    : address{address}
    , name{name}
    , metadata{metadata} {
    for (u8 index{1}; index <= metadata.fieldCount; index++)
        parameters.emplace_back(Parameter{index});
}

void Device::Update(const std::string& pName, DeviceMetadata pMetadata) {
    if (name != pName || metadata != pMetadata) {
        name = pName;
        metadata = pMetadata;
        
        parameters.clear();
        for (u8 index{1}; index <= metadata.fieldCount; index++)
            parameters.emplace_back(Parameter{index});

        parameterWrites.clear();
        syncData.reset();
    }
}

Parameter* Device::FindParameter(u8 fieldIndex) {
    auto it{std::find_if(parameters.begin(), parameters.end(), [&](const Parameter& parameter) { return parameter.fieldIndex == fieldIndex; })};
    if (it == parameters.end())
        return nullptr;
    return &*it;
}

void Device::WriteParameter(u8 fieldIndex, ParameterWriteFrame::WriteValue value) {
    Parameter* parameter{FindParameter(fieldIndex)};
    if (!parameter || !parameter->complete)
        return;

    parameterWrites.emplace_back(ParameterWriteFrame{fieldIndex, value, address});

    if (parameter->type == ParameterType::Command) {
        parameter->activeCommand = true;
        parameter->Reset();
        return;
    }

    // We need to reset siblings along with the parent, when a parameter is written to since their values might be interdependent.
    // Any children, or siblings that are subfolders or commands will not be reset since they are not expected to have interdependent values.
    u8 parentIndex{parameter->parentIndex};
    for (Parameter& parameter : parameters)
        if (parameter.fieldIndex == fieldIndex || (parameter.parentIndex == parentIndex && parameter.type != ParameterType::Folder && parameter.type != ParameterType::Command))
            parameter.Reset();
}

std::optional<FrameV> Device::TryGetTxFrame() {
    if (!parameterWrites.empty()) {
        FrameV frame{parameterWrites.front()};
        parameterWrites.pop_front();
        return frame;
    }

    for (Parameter& parameter : parameters) {
        constexpr auto ParameterResendTimeout{std::chrono::milliseconds(100)};
        if (!parameter.complete) {
            if (parameter.requestedChunk && (std::chrono::steady_clock::now() - parameter.requestTime) < ParameterResendTimeout)
                continue;
            parameter.requestedChunk = true;
            parameter.requestTime = std::chrono::steady_clock::now();
            return ParameterReadFrame{parameter.fieldIndex, parameter.chunkIndex, address};
        }
    }

    return std::nullopt;
}
}