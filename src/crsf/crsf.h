/* SPDX-License-Identifier: MPL-2.0
 * Copyright © Mark Collins
 */
#pragma once

#include <list>

#include "frame.h"
#include "parameter.h"
#include "protocol.h"

#include <serial/serial.h>

namespace crsf {
class Interface {
  private:
    SerialPort serial;
    FrameData oldFrame{}; //!< A copy of the previous frame, used to prevent duplicate frames from being processed.

  public:
    Interface(std::string port, size_t baud);

    /**
      * @return Whether the interface is active, i.e. the serial port is open. This should be checked after exceptions.
     */
    bool IsActive() noexcept;

    /**
     * @brief Attempts to reset the device by asserting the DTR and RTS lines.
     */
    void ResetDevice() noexcept;

    /**
     * @brief Sends a frame over the CRSF interface.
     */
    void SendFrame(const FrameV& frame);

    /**
     * @brief Receives a frame from the CRSF interface.
     * @param timeout The time to wait for a frame before returning an empty optional.
     * @return The received frame, or an empty optional if no frame was received.
     */
    std::optional<FrameV> ReceiveFrame(std::chrono::time_point<std::chrono::steady_clock> timeout);
};

struct Device {
    Address address;
    std::string name;
    DeviceMetadata metadata;
    std::vector<Parameter> parameters;
    std::list<ParameterWriteFrame> parameterWrites;
    std::optional<OpenTxSyncData> syncData;

    Device(Address address, const std::string& name, DeviceMetadata metadata);

    /**
     * @brief Updates the device metadata, resetting all state if there are changes.
     */
    void Update(const std::string& name, DeviceMetadata metadata);

    /**
     * @brief Finds a parameter by its field index, returning nullptr if it does not exist. 
     */
    Parameter* FindParameter(u8 fieldIndex);

    /**
     * @brief Writes a parameter value to the device, resetting dependent parameters.
     */
    void WriteParameter(u8 fieldIndex, ParameterWriteFrame::WriteValue value);

    /**
     * @brief Attempts to get a queued frame to transmit over the CRSF interface.
     */
    std::optional<FrameV> TryGetTxFrame();
};
} // namespace crsf