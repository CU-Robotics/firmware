#include <cassert>
#include <cstring>
#include <iostream>
#include "comms/data/hive_data.hpp"
#include "comms/data/firmware_data.hpp"

extern "C" void reset_teensy() { assert(false && "Unexpected reconfiguration"); }

int main() {
    Comms::HiveData hive;
    Cfg::ConfigStart start;
    start.num_config_sections = 1;
    hive.set_data(&start);
    assert(!hive.config.is_configured());
    // Independent bytes describing Gerald's configuration, as emitted by Hive.
    const uint8_t wire[] = {16, 0, 34, 0, 1, 0, 0, 0, 3, 0, 0, 0, 10, 0, 0, 0};
    Cfg::Vn100Imu config;
    std::memcpy(&config, wire, sizeof(config));
    hive.set_data(&config);
    assert(hive.config.is_configured());
    assert(hive.config.num_sections_received == 1);
    assert(hive.config.vn100_imus.size() == 1);
    const auto& received = hive.config.vn100_imus[0];
    assert(received.imu_name == Cfg::SensorName::YawVn100Imu);
    assert(received.hardware_serial_port == Cfg::HardwareSerialPort::Serial3);
    assert(&Cfg::try_grab_hw_serial_port(received.hardware_serial_port) == &Serial3);

    Comms::FirmwareData firmware;
    Vn100SensorData sample(Cfg::SensorName::YawVn100Imu);
    sample.valid = 3;
    sample.imu_time = 100;
    sample.accel[0] = 9.5f;
    firmware.set_data(&sample);
    sample.imu_time = 200;
    sample.accel[0] = 10.5f;
    firmware.set_data(&sample);
    assert(firmware.vn100_sensor_data_map.size() == 1);
    const auto& stored = firmware.vn100_sensor_data_map.at(Cfg::SensorName::YawVn100Imu);
    assert(stored.imu_time == 200 && stored.accel[0] == 10.5f);
    std::cout << "VN100 firmware configuration and telemetry routing tests passed\n";
}
