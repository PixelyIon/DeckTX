/* SPDX-License-Identifier: MIT
 * Copyright © Mark Collins
 */
#pragma once

#include <span>
#include <string>
#include <vector>
#include <chrono>

#if defined(_WIN32)
    #include <windows.h>

    #define SERIAL_WIN 1
#elif defined(__linux__)
    #define SERIAL_LINUX 1
#else
    #error "Unsupported platform"
#endif

#include <common/types.h>

struct SerialPortInfo {
    std::string devicePath; //!< The platform specific device path, e.g. "/dev/ttyUSB0" or "COM1".
    std::string friendlyName; //!< Human readable name of the serial device, if available.
    std::string hardwareId; //!< The hardware ID of the device, this might be a VID/PID of the device or other hardware identifiers.
};

/**
 * @brief List all serial ports currently available on the system.
 */
std::vector<SerialPortInfo> ListSerialPorts();

/**
 * @brief An abstraction over an active serial port, with methods for reading and writing to it.
 */
class SerialPort {
  private:
    #if SERIAL_WIN
        HANDLE handle{INVALID_HANDLE_VALUE};
    #elif SERIAL_LINUX
        int fd{-1};
    #endif

  public:
    /**
     * @brief Open a serial port.
     * @param devicePath The platform specific device path, e.g. "/dev/ttyUSB0" or "COM1".
     * @param baudRate The baud rate to use for the serial port.
     * @note The serial port is always opened with 8 data bits, no parity, and 1 stop bit.
     */
    SerialPort(std::string devicePath, size_t baudRate);

    ~SerialPort();

    /**
     * @return If the serial port is currently open, connection status may be disrupted due to external factors (such as unplugging the device) so this should be checked after exceptions.
     */
    bool IsOpen();

    /**
     * @brief Read a single byte from the serial port.
     * @param timeout The time to wait for a byte before returning an empty optional.
     * @return The byte read, or an empty optional if no byte was read.
     */
    std::optional<u8> ReadByte(std::chrono::time_point<std::chrono::steady_clock> timeout = std::chrono::steady_clock::now() + std::chrono::seconds(1));

    /**
     * @brief Read a buffer of bytes from the serial port.
     * @note This will block until the buffer is filled, early returns are only possible via exceptions.
     */
    void Read(std::span<u8> buffer);

    template <typename T>
    void Read(std::span<T> buffer) {
        Read(std::span<u8>(reinterpret_cast<u8*>(buffer.data()), buffer.size() * sizeof(T)));
    }

    /**
     * @brief Write a buffer of bytes to the serial port.
     * @note This will block until the buffer is written, early returns are only possible via exceptions.
     */
    void Write(std::span<u8> buffer);

    void WriteByte(u8 byte) {
        Write(std::span<u8>(&byte, sizeof(byte)));
    }

    void SetDTR(bool state);

    void SetRTS(bool state);
};