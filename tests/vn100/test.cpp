#include <cassert>
#include <iostream>
#include <type_traits>
#include <vector>

#include "sensors/vn100.hpp"
#include "comms/data/sendable.hpp"
#include "comms/data/vn100_sensor_data.hpp"
#include <fstream>

using Bytes = std::vector<uint8_t>;

template<class T> void append(Bytes &bytes, T value) {
    const auto *start = reinterpret_cast<const uint8_t *>(&value);
    bytes.insert(bytes.end(), start, start + sizeof(value));
}

void finish(Bytes &bytes) {
    const uint16_t crc = vn::calculate_crc(bytes.data() + 1, bytes.size() - 1);
    bytes.push_back(crc >> 8);
    bytes.push_back(crc & 0xff);
}

Bytes imu_packet(uint64_t time, float x) {
    Bytes bytes{0xfa, 0x06, 0x01, 0x00, 0x00, 0x0e};
    append(bytes, time);
    for (float value : {x, 2.f, 3.f, 4.f, 5.f, 6.f}) append(bytes, value);
    append(bytes, uint16_t{8}); // acceleration X saturated
    finish(bytes);
    return bytes;
}

Bytes attitude_packet() {
    Bytes bytes{0xfa, 0x16, 0x01, 0x00, 0x30, 0x01, 0x86, 0x00};
    append(bytes, uint64_t{200});
    // Temperature, pressure, magnetic field, YPR, quaternion, NED acceleration.
    for (float value : {25.f, 101.f, 1.f, 2.f, 3.f, 90.f, -10.f, 20.f,
                       0.f, 0.f, 0.f, 1.f, 7.f, 8.f, 9.f}) append(bytes, value);
    finish(bytes);
    return bytes;
}

void feed(HardwareSerial &port, const Bytes &bytes) {
    port.input.insert(port.input.end(), bytes.begin(), bytes.end());
}

int main() {
    static_assert(!std::is_copy_constructible_v<vn100>);
    static_assert(!std::is_move_constructible_v<vn100>);
    static_assert(static_cast<uint16_t>(Comms::TypeLabel::LsmSensorData) == 6);
    static_assert(static_cast<uint16_t>(Comms::TypeLabel::TestLatencyData) == 32);
    static_assert(static_cast<uint16_t>(Comms::TypeLabel::Vn100SensorData) == 33);
    static_assert(static_cast<uint16_t>(Comms::TypeLabel::Vn100ImuConfig) == 34);
    static_assert(static_cast<uint32_t>(Cfg::SensorName::YawVn100Imu) == 10);
    vn100 default_sensor;
    assert(Serial3.begins == 0);
    Serial3.respond = true;
    default_sensor.init();
    assert(default_sensor.is_initialized() && Serial3.begins > 0);

    HardwareSerial port1, port2;
    vn100 first(port1), second(port2);
    assert(!first.is_initialized());
    assert(!first.get_data().imu_valid && !first.get_data().attitude_valid);
    first.read();
    first.send_to_comms();
    assert(Comms::comms_layer.queued.empty());
    assert(port1.writes == 0);
    first.init(); // No device replies: report failure and permit an explicit retry.
    assert(!first.is_initialized());
    auto writes = port1.writes;
    assert(writes > 0);
    first.read();
    assert(port1.writes == writes); // No blocking retry in the polling loop.

    port1.respond = true;
    port1.device_baud = 115200; // Exercise probing and baud change on explicit retry.
    first.init();
    assert(first.is_initialized());
    first.send_to_comms();
    assert(Comms::comms_layer.queued.empty());
    assert(port1.baud == 921600 && port1.device_baud == 921600);
    writes = port1.writes;
    port2.respond = true;
    second.init();
    assert(second.is_initialized());

    // Verify the configuration sent over the public transport API.
    auto sent = [](const HardwareSerial &port, const std::string &command) {
        return std::find(port.commands.begin(), port.commands.end(), command) != port.commands.end();
    };
    assert(sent(port1, "VNASY,0"));
    assert(sent(port1, "VNWRG,75,2,2,06,0001,0E00"));
    assert(sent(port1, "VNWRG,76,2,8,16,0001,0130,0086"));
    assert(sent(port1, "VNWRG,77,0,0,00"));
    assert(port1.commands.back() == "VNASY,1");
    const auto imu = imu_packet(100, 1.f);
    feed(port1, Bytes(imu.begin(), imu.begin() + 12));
    first.read();
    assert(!first.get_data().imu_valid);
    feed(port1, Bytes(imu.begin() + 12, imu.end()));
    first.read();
    assert(first.get_data().imu_valid);
    assert(first.get_data().imu_time.time_startup == 100);
    assert(first.get_data().accel.acc[0] == 1.f);
    assert(first.get_data().angular_rate.gyro[2] == 6.f);
    assert(first.get_data().saturation.acc_x == 1);
    assert(!first.get_data().attitude_valid);
    assert(!second.get_data().imu_valid);
    first.send_to_comms();
    Vn100SensorData partial;
    std::memcpy(&partial, Comms::comms_layer.queued.back().data(), sizeof(partial));
    assert(partial.valid == 1 && partial.attitude_time == 0);
    assert(partial.accel[0] == 1.f && partial.temperature == 0.f);

    feed(port2, imu_packet(300, 42.f));
    second.read();
    assert(second.get_data().accel.acc[0] == 42.f);
    assert(first.get_data().accel.acc[0] == 1.f);

    feed(port1, attitude_packet());
    first.read();
    const auto &data = first.get_data();
    assert(data.attitude_valid && data.attitude_time.time_startup == 200);
    assert(data.temperature.temperature == 25.f && data.pressure.pressure == 101.f);
    assert(data.mag.mag[2] == 3.f);
    assert(data.ypr.yaw == 90.f && data.ypr.pitch == -10.f && data.ypr.roll == 20.f);
    assert(data.quaternion.quaternion[3] == 1.f && data.lin_accel_ned.lin_accel_ned[2] == 9.f);
    assert(data.imu_time.time_startup == 100); // Independent packet timestamps.

    auto corrupt = imu_packet(400, 99.f);
    corrupt.back() ^= 1;
    feed(port1, corrupt);
    first.read();
    first.read(); // Empty input also preserves the last valid sample.
    assert(data.imu_time.time_startup == 100 && data.accel.acc[0] == 1.f);

    feed(port1, imu_packet(500, 11.f));
    first.read();
    assert(data.imu_time.time_startup == 500 && data.accel.acc[0] == 11.f);
    assert(data.attitude_time.time_startup == 200);
    first.init(); // Already initialized: do not repeat UART setup.
    assert(port1.writes == writes);
    // Configuration failure must leave polling inactive; a retry can recover.
    HardwareSerial failing_port;
    failing_port.respond = true;
    failing_port.fail_command = "VNWRG,76";
    vn100 failing(failing_port);
    failing.init();
    assert(!failing.is_initialized());
    const auto failed_writes = failing_port.writes;
    feed(failing_port, imu_packet(600, 12.f));
    failing.read();
    assert(!failing.get_data().imu_valid && failing_port.writes == failed_writes);
    failing_port.fail_command.clear();
    failing.init();
    assert(failing.is_initialized());
    feed(failing_port, imu_packet(700, 13.f));
    failing.read();
    assert(failing.get_data().imu_time.time_startup == 700);
    assert(first.get_data().imu_time.time_startup == 500);
    Cfg::Vn100Imu config;
    config.hardware_serial_port = Cfg::HardwareSerialPort::Serial5;
    config.imu_name = Cfg::SensorName::YawVn100Imu;
    const auto serial3_writes = Serial3.writes;
    Serial5.respond = true;
    vn100 configured(config);
    assert(Serial5.begins == 0);
    configured.init();
    assert(configured.is_initialized() && Serial5.begins > 0);
    assert(Serial3.writes == serial3_writes); // Configured UART, not the default.
    feed(Serial5, attitude_packet());
    configured.read();
    configured.send_to_comms();
    Vn100SensorData sample;
    std::memcpy(&sample, Comms::comms_layer.queued.back().data(), sizeof(sample));
    assert(sample.valid == 2 && sample.imu_time == 0 && sample.accel[0] == 0);
    feed(Serial5, imu_packet(0x0102030405060708ULL, 11.f));
    configured.read();
    configured.send_to_comms();
    const auto& bytes = Comms::comms_layer.queued.back();
    assert(bytes.size() == 120);
    std::memcpy(&sample, bytes.data(), sizeof(sample));
    assert(sample.type_label == Comms::TypeLabel::Vn100SensorData);
    assert(sample.physical_medium == Comms::PhysicalMedium::Ethernet);
    assert(sample.priority == Comms::Priority::Medium);
    assert(sample.imu_name == Cfg::SensorName::YawVn100Imu);
    assert(sample.valid == 3 && sample.saturation == 8);
    assert(sample.imu_time == 0x0102030405060708ULL && sample.attitude_time == 200);
    assert(sample.accel[0] == 11.f && sample.gyro[2] == 6.f);
    assert(sample.temperature == 25.f && sample.pressure == 101.f);
    assert(sample.mag[2] == 3.f && sample.ypr[1] == -10.f);
    assert(sample.quaternion[3] == 1.f && sample.lin_accel_ned[2] == 9.f);
    // Golden wire fixture also consumed by the Hive integration tests.
    std::ofstream telemetry("build/tests/vn100_telemetry.bin", std::ios::binary);
    telemetry.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    assert(telemetry.good());
    std::ofstream cfg("build/tests/vn100_config.bin", std::ios::binary);
    cfg.write(reinterpret_cast<const char*>(&config), sizeof(config));
    assert(cfg.good());
    config.hardware_serial_port = Cfg::HardwareSerialPort::UnsetHardwareSerialPort;
    bool rejected = false;
    try { vn100 invalid(config); } catch (const std::runtime_error&) { rejected = true; }
    assert(rejected);
    std::cout << "VN100 packet, UART, telemetry and wire layout tests passed\n";
}
