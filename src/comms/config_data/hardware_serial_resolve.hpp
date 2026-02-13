#include "sensor.hpp"
#include <HardwareSerial.h>


namespace NewConfig {

    std::optional<HardwareSerial*> resolve_hardware_serial_port(HardwareSerialPort port) {
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
}