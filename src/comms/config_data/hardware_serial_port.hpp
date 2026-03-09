#pragma once
#include <HardwareSerial.h>
#include <optional>
#include "utils/safety.hpp"

namespace Cfg {

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

inline HardwareSerial& try_grab_hw_serial_port(Cfg::HardwareSerialPort port) {
  auto port_opt = Cfg::resolve_hardware_serial_port(port);
  if (!port_opt.has_value()) {
    safety::safety_procedure("D200LD14P: failed to resolve hardware serial port %u", static_cast<uint32_t>(port));
  }
  return *port_opt.value();
}

}