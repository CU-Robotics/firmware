#pragma once
#include <HardwareSerial.h>
#include <optional>
#include "utils/safety.hpp"

namespace Cfg {
/// @brief This enum represents the different hardware serial ports that can be configured for use in the config. 
// The values of the enum correspond to the actual hardware serial ports on the microcontroller, so they should not be changed.
enum class HardwareSerialPort : uint32_t {
    UnsetHardwareSerialPort,
    Serial1,
    Serial2,
    Serial3,
    Serial4,
    Serial5,
    Serial6,
    Serial7,
    Serial8,
};
/// @brief Convert a HardwareSerialPort enum value to a pointer to the corresponding HardwareSerial object.
/// @param port The HardwareSerialPort enum value to convert to a HardwareSerial pointer.
/// @return A pointer to the corresponding HardwareSerial object, or std::nullopt if the port is UnsetHardwareSerialPort or if the port does not correspond to a valid hardware serial port on the microcontroller.
inline std::optional<HardwareSerial*> resolve_hardware_serial_port(HardwareSerialPort port) {
    switch(port) {
        case HardwareSerialPort::Serial1:
            return &Serial1;
        case HardwareSerialPort::Serial2:
            return &Serial2;
        case HardwareSerialPort::Serial3:
            return &Serial3;
        case HardwareSerialPort::Serial4:
            return &Serial4;
        case HardwareSerialPort::Serial5:
            return &Serial5;
        case HardwareSerialPort::Serial6:
            return &Serial6;
        case HardwareSerialPort::Serial7:
            return &Serial7;
        case HardwareSerialPort::Serial8:
            return &Serial8;
        default:
            return std::nullopt;
    }
}
/// @brief Helper function to get a reference to a HardwareSerial object corresponding to a HardwareSerialPort enum value. 
/// @note This will trigger a safety procedure if the port is UnsetHardwareSerialPort or if the port does not correspond to a valid hardware serial port on the microcontroller.
/// @param port The HardwareSerialPort enum value to get the HardwareSerial reference for.
/// @return A reference to the corresponding HardwareSerial object.
inline HardwareSerial& try_grab_hw_serial_port(Cfg::HardwareSerialPort port) {
  auto port_opt = Cfg::resolve_hardware_serial_port(port);
  if (!port_opt.has_value()) {
    safety::safety_procedure("D200LD14P: failed to resolve hardware serial port %u", static_cast<uint32_t>(port));
  }
  return *port_opt.value();
}
}