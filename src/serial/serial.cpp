/* SPDX-License-Identifier: MIT
 * Copyright © Craig Lilley
 * Copyright © Yan Naing Aye
 * Copyright © Mark Collins
 * Heavily derivative of wjwwood/serial, see https://github.com/wjwwood/serial/tree/69e0372cf0d3796e84ce9a09aff1d74496f68720/src/impl/list_ports
 * Heavily derivative of yan9a/serial, see https://github.com/yan9a/serial/blob/7346457660b192a38660f53e0ea11565719682f4/ceSerial.h
 */

#include "serial.h"

#if SERIAL_WIN
    #include <devguid.h>
    #include <initguid.h>

    #include <setupapi.h>
    #include <tchar.h>
#elif SERIAL_LINUX
    #include <filesystem>
    #include <fstream>
    #include <regex>

    #include <unistd.h>
    #include <fcntl.h>
    #include <termios.h>
    #include <sys/ioctl.h>
    #include <sys/select.h>
    #include <errno.h>
#else
    #error "Unsupported platform"
#endif

#include <common/error.h>
#include <common/scope.h>

std::vector<SerialPortInfo> ListSerialPorts() {
    std::vector<SerialPortInfo> ports;

#if SERIAL_WIN

    HDEVINFO deviceInfoSet{SetupDiGetClassDevs((const GUID*)&GUID_DEVCLASS_PORTS, NULL, NULL, DIGCF_PRESENT)};
    if (deviceInfoSet == INVALID_HANDLE_VALUE)
        return ports;
    SCOPE_EXIT(SetupDiDestroyDeviceInfoList(deviceInfoSet));

    SP_DEVINFO_DATA deviceInfoData{.cbSize = sizeof(SP_DEVINFO_DATA)};

    for (DWORD deviceInfoSetIndex{}; SetupDiEnumDeviceInfo(deviceInfoSet, deviceInfoSetIndex, &deviceInfoData); deviceInfoSetIndex++) {
        // Get device path.
        std::string devicePath;
        {
            HKEY hkey{SetupDiOpenDevRegKey(
                deviceInfoSet,
                &deviceInfoData,
                DICS_FLAG_GLOBAL,
                0,
                DIREG_DEV,
                KEY_READ)};
            SCOPE_EXIT(RegCloseKey(hkey));

            DWORD devicePathLength{};
            LONG returnCode{RegQueryValueExA(
                hkey,
                _T("PortName"),
                NULL,
                NULL,
                NULL,
                &devicePathLength)};

            if (returnCode != EXIT_SUCCESS || devicePathLength == 0)
                continue;

            devicePath.resize(devicePathLength - 1); // String length includes null terminator.

            returnCode = RegQueryValueExA(
                hkey,
                _T("PortName"),
                NULL,
                NULL,
                (LPBYTE)devicePath.data(),
                &devicePathLength);

            if (returnCode != EXIT_SUCCESS)
                continue;
        }

        // Ignore parallel ports.
        if (devicePath.find("LPT") != std::string::npos)
            continue;

        // Get friendly name and hardware ID.
        auto getDeviceRegistryString = [&](DWORD property) -> std::string {
            DWORD length{};
            SetupDiGetDeviceRegistryProperty(
                deviceInfoSet,
                &deviceInfoData,
                property,
                NULL,
                NULL,
                0,
                &length);

            if (length == 0)
                return {};

            std::string value;
            value.resize(length);

            SetupDiGetDeviceRegistryProperty(
                deviceInfoSet,
                &deviceInfoData,
                property,
                NULL,
                (PBYTE)value.data(),
                length,
                &length);

            return value;
        };

        ports.emplace_back(SerialPortInfo{
            devicePath,
            getDeviceRegistryString(SPDRP_FRIENDLYNAME),
            getDeviceRegistryString(SPDRP_HARDWAREID),
        });
    }

#elif SERIAL_LINUX

    const std::string sysClassTTY{"/sys/class/tty/"};
    const std::regex ttyPattern{"^ttyS|^ttyUSB|^ttyACM"};

    for (const auto& entry : std::filesystem::directory_iterator(sysClassTTY)) {
        std::string portName{entry.path().filename().string()};
        if (!std::regex_search(portName, ttyPattern))
            continue;

        std::string devicePath{"/dev/" + portName};
        std::string friendlyName;
        std::string hardwareID;

        std::ifstream ueventFile(entry.path() / "device" / "uevent");
        if (ueventFile) {
            std::string line;
            while (std::getline(ueventFile, line)) {
                if (line.find("PRODUCT=") != std::string::npos)
                    hardwareID = line.substr(line.find('=') + 1);
                if (line.find("DRIVER=") != std::string::npos)
                    friendlyName = line.substr(line.find('=') + 1);
            }
        }

        ports.emplace_back(SerialPortInfo{
            devicePath,
            friendlyName.empty() ? portName : friendlyName,
            hardwareID,
        });
    }

#endif

    return ports;
}

#if SERIAL_WIN
constexpr COMMTIMEOUTS DefaultTimeouts{
    .ReadIntervalTimeout = 0,
    .ReadTotalTimeoutMultiplier = 0,
    .ReadTotalTimeoutConstant = 0,
    .WriteTotalTimeoutMultiplier = 0,
    .WriteTotalTimeoutConstant = 0,
};
#endif

SerialPort::SerialPort(std::string devicePath, size_t baudRate) {
#if SERIAL_WIN

    handle = CreateFile(
        _T(devicePath.c_str()),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
        NULL);

    if (handle == INVALID_HANDLE_VALUE)
        throw Exception("Failed to open serial port ({}): {}", devicePath, GetLastError());

    DCB dcbSerialParams{.DCBlength = sizeof(dcbSerialParams)};

    if (!GetCommState(handle, &dcbSerialParams))
        throw Exception("Failed to get serial port state ({}): {}", devicePath, GetLastError());

    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.Parity = NOPARITY;
    dcbSerialParams.StopBits = ONESTOPBIT;

    dcbSerialParams.fOutxCtsFlow = false;
    dcbSerialParams.fOutxDsrFlow = false;
    dcbSerialParams.fOutX = false;
    dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;

    if (!SetCommState(handle, &dcbSerialParams))
        throw Exception("Failed to set serial port state ({}): {}", devicePath, GetLastError());

    if (!SetCommTimeouts(handle, const_cast<LPCOMMTIMEOUTS>(&DefaultTimeouts)))
        throw Exception("Failed to set serial port timeouts ({}): {}", devicePath, GetLastError());

#elif SERIAL_LINUX

    fd = open(devicePath.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
        throw Exception("Failed to open serial port ({}): {}", devicePath, strerror(errno));

    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0)
        throw Exception("Failed to get serial port state ({}): {}", devicePath, strerror(errno));

    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
        throw Exception("Failed to set serial port state ({}): {}", devicePath, strerror(errno));

#endif
}

SerialPort::~SerialPort() {
#if SERIAL_WIN

    if (handle != INVALID_HANDLE_VALUE)
        CloseHandle(handle);

#elif SERIAL_LINUX

    if (fd != -1)
        close(fd);

#endif
}

bool SerialPort::IsOpen() {
#if SERIAL_WIN

    DWORD errors{};
    COMSTAT comStat{};
    if (!ClearCommError(handle, &errors, &comStat) || (errors & (CE_OVERRUN | CE_RXOVER | CE_RXPARITY | CE_FRAME | CE_BREAK)))
        return false;

    return true;

#elif SERIAL_LINUX

    return fcntl(fd, F_GETFD) != -1 || errno != EBADF;

#endif
}

#if SERIAL_WIN
struct RaiiHandle {
    ~RaiiHandle() {
        if (handle != INVALID_HANDLE_VALUE)
            CloseHandle(handle);
    }

    HANDLE handle{INVALID_HANDLE_VALUE};
};

static thread_local RaiiHandle OverlapEvent{};

static HANDLE GetOverlapEvent() {
    if (OverlapEvent.handle == INVALID_HANDLE_VALUE) {
        OverlapEvent.handle = CreateEvent(NULL, TRUE, FALSE, NULL);
        if (!OverlapEvent.handle)
            throw Exception("Failed to create overlapped event: {}", GetLastError());
    }
    return OverlapEvent.handle;
}

size_t ReadSerialFile(HANDLE handle, void* buffer, size_t size, std::chrono::time_point<std::chrono::steady_clock> timeout = {}) {
    if (timeout != std::chrono::time_point<std::chrono::steady_clock>{} && std::chrono::steady_clock::now() >= timeout)
        return 0;

    OVERLAPPED overlapped{
        .hEvent = GetOverlapEvent(),
    };

    if (!ReadFile(handle, buffer, size, nullptr, &overlapped)) {
        if (GetLastError() != ERROR_IO_PENDING)
            throw Exception("Failed to read from serial port: {}", GetLastError());
    }

    size_t timeoutDword{INFINITE};
    if (timeout != std::chrono::time_point<std::chrono::steady_clock>{}) {
        auto timeoutMs{std::chrono::duration_cast<std::chrono::milliseconds>(timeout - std::chrono::steady_clock::now()).count()};
        timeoutDword = timeoutMs <= 0 ? 0 : static_cast<DWORD>(timeoutMs);
    }

    bool cancelled{};
    if (WaitForSingleObject(overlapped.hEvent, timeoutDword) != WAIT_OBJECT_0) {
        if (!CancelIoEx(handle, &overlapped))
            throw Exception("Failed to cancel I/O: {}", GetLastError());
        cancelled = true;
    }

    DWORD bytesRead{};
    if (!GetOverlappedResult(handle, &overlapped, &bytesRead, TRUE)) {
        if (cancelled)
            return 0;
        throw Exception("Failed to get overlapped result: {}", GetLastError());
    }

    return bytesRead;
}

constexpr DWORD MaxTimeout{std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::hours(1)).count()};
#endif

std::optional<u8> SerialPort::ReadByte(std::chrono::time_point<std::chrono::steady_clock> timeout) {
    if (std::chrono::steady_clock::now() >= timeout)
        return std::nullopt;

    u8 byte{};
#if SERIAL_WIN

    size_t bytesRead{ReadSerialFile(handle, &byte, 1, timeout)};
    if (bytesRead == 0)
        return std::nullopt;

#elif SERIAL_LINUX

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);

    struct timeval tv {
        .tv_sec = std::chrono::duration_cast<std::chrono::seconds>(timeout - std::chrono::steady_clock::now()).count(),
        .tv_usec = std::chrono::duration_cast<std::chrono::microseconds>(timeout - std::chrono::steady_clock::now()).count() % 1000000,
    };

    if (select(fd + 1, &readfds, NULL, NULL, &tv) == -1)
        throw Exception("Failed to wait for serial port: {}", strerror(errno));

    if (!FD_ISSET(fd, &readfds))
        return std::nullopt;

    if (read(fd, &byte, 1) != 1)
        throw Exception("Failed to read from serial port: {}", strerror(errno));

#endif

    return byte;
}

void SerialPort::Read(std::span<u8> buffer) {
#if SERIAL_WIN

    size_t bytesRead{ReadSerialFile(handle, buffer.data(), buffer.size())};
    if (bytesRead != buffer.size())
        throw Exception("Failed to read from serial port: expected {}, got {}", buffer.size(), bytesRead);

#elif SERIAL_LINUX

    if (read(fd, buffer.data(), buffer.size()) != buffer.size())
        throw Exception("Failed to read from serial port: {}", strerror(errno));

#endif
}

void SerialPort::Write(std::span<u8> buffer) {
#if SERIAL_WIN

    OVERLAPPED overlapped{
        .hEvent = GetOverlapEvent(),
    };

    if (!WriteFile(handle, buffer.data(), buffer.size(), nullptr, &overlapped)) {
        if (GetLastError() != ERROR_IO_PENDING)
            throw Exception("Failed to write to serial port: {}", GetLastError());
    }

    DWORD bytesWritten{};
    if (!GetOverlappedResult(handle, &overlapped, &bytesWritten, TRUE))
        throw Exception("Failed to get overlapped result: {}", GetLastError());

    if (bytesWritten != buffer.size())
        throw Exception("Failed to write to serial port: expected {}, got {}", buffer.size(), bytesWritten);

#elif SERIAL_LINUX

    if (write(fd, buffer.data(), buffer.size()) != buffer.size())
        throw Exception("Failed to write to serial port: {}", strerror(errno));

#endif
}

void SerialPort::SetDTR(bool state) {
#if SERIAL_WIN

    if (!EscapeCommFunction(handle, state ? SETDTR : CLRDTR))
        throw Exception("Failed to set DTR: {}", GetLastError());

#elif SERIAL_LINUX

    int status{state ? TIOCM_DTR : 0};
    if (ioctl(fd, TIOCMSET, &status) == -1)
        throw Exception("Failed to set DTR: {}", strerror(errno));

#endif
}

void SerialPort::SetRTS(bool state) {
#if SERIAL_WIN

    if (!EscapeCommFunction(handle, state ? SETRTS : CLRRTS))
        throw Exception("Failed to set RTS: {}", GetLastError());

#elif SERIAL_LINUX

    int status{state ? TIOCM_RTS : 0};
    if (ioctl(fd, TIOCMSET, &status) == -1)
        throw Exception("Failed to set RTS: {}", strerror(errno));

#endif
}