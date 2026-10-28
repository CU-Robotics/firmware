#include <Arduino.h>
#include "sensors/buff_encoder.hpp"
#include <unity.h>

#ifdef UNIT_TEST
#include "sensors/vn100.hpp"
#include <bit>
#include <initializer_list>
#include <string>
#include <vector>
#endif

// OG dummy sensor rocko tests

class DummySensor : public Sensor {
  public:
    void read() override { read_called_ = true; }
    void init() override { init_called_ = true; }
    void send_to_comms() const override { send_called_ = true; }

    bool was_read_called() const { return read_called_; }
    bool was_init_called() const { return init_called_; }
    bool was_send_to_comms_called() const { return send_called_; }

  private:
    bool read_called_ = false;
    bool init_called_ = false;
    mutable bool send_called_ = false;
};

void test_sensor_read_called() {
    DummySensor sensor;
    sensor.read();
    TEST_ASSERT_TRUE(sensor.was_read_called());
}

void test_sensor_init_called() {
    DummySensor sensor;
    sensor.init();
    TEST_ASSERT_TRUE(sensor.was_init_called());
}

void test_send_to_comms_called() {
    DummySensor sensor;
    sensor.send_to_comms();
    TEST_ASSERT_TRUE(sensor.was_send_to_comms_called());
}

void test_sensor_life_cycle() {
    DummySensor sensor;
    sensor.init();
    sensor.read();
    sensor.send_to_comms();

    TEST_ASSERT_TRUE(sensor.was_init_called());
    TEST_ASSERT_TRUE(sensor.was_read_called());
    TEST_ASSERT_TRUE(sensor.was_send_to_comms_called());
}

// Shared buff encoder SPI bus: MISO=12, MOSI=11, SCK=13

static Cfg::BuffEncoder make_yaw_encoder_cfg() {
    Cfg::BuffEncoder cfg{};
    cfg.spi_cs = 37;
    cfg.spi_miso = 12;
    cfg.spi_mosi = 11;
    cfg.spi_sck  = 13;
    cfg.encoder_name = Cfg::SensorName::YawBuffEncoder;
    return cfg;
}

static Cfg::BuffEncoder make_pitch_encoder_cfg() {
    Cfg::BuffEncoder cfg{};
    cfg.spi_cs = 28;
    cfg.spi_miso = 12;
    cfg.spi_mosi = 11;
    cfg.spi_sck = 13;
    cfg.encoder_name = Cfg::SensorName::PitchBuffEncoder;
    return cfg;
}

static Cfg::BuffEncoder make_feeder_encoder_cfg() {
    Cfg::BuffEncoder cfg{};
    cfg.spi_cs = 9;
    cfg.spi_miso = 12;
    cfg.spi_mosi = 11;
    cfg.spi_sck = 13;
    cfg.encoder_name = Cfg::SensorName::FeederBuffEncoder;
    return cfg;
}

void test_yaw_encoder_init() {
    auto cfg = make_yaw_encoder_cfg();
    BuffEncoder encoder(cfg);
    encoder.init();
    TEST_ASSERT_EQUAL(Cfg::SensorName::YawBuffEncoder, encoder.get_name());
}

void test_pitch_encoder_init() {
    auto cfg = make_pitch_encoder_cfg();
    BuffEncoder encoder(cfg);
    encoder.init();
    TEST_ASSERT_EQUAL(Cfg::SensorName::PitchBuffEncoder, encoder.get_name());
}

void test_feeder_encoder_init() {
    auto cfg = make_feeder_encoder_cfg();
    BuffEncoder encoder(cfg);
    encoder.init();
    TEST_ASSERT_EQUAL(Cfg::SensorName::FeederBuffEncoder, encoder.get_name());
}

/*
void test_yaw_encoder_read_returns_finite_float() {
    auto cfg = make_yaw_encoder_cfg();
    BuffEncoder encoder(cfg);
    encoder.init();
    encoder.read();
    float angle = encoder.get_angle();
    TEST_ASSERT_FALSE(isnan(angle));
    TEST_ASSERT_FALSE(isinf(angle));
}
*/

void test_pitch_encoder_read_returns_finite_float() {
    auto cfg = make_pitch_encoder_cfg();
    BuffEncoder encoder(cfg);
    encoder.init();
    encoder.read();
    float angle = encoder.get_angle();
    TEST_ASSERT_FALSE(isnan(angle));
    TEST_ASSERT_FALSE(isinf(angle));
}

#ifdef UNIT_TEST
// Software-only VN100 tests: the production driver/parser talk to an in-memory
// device with scripted replies. No physical UART, sensor, or wall-clock waits.
namespace {
using Packet = std::vector<uint8_t>;

// Build fixtures independently of the production message creation/CRC code.
std::string vn100_ascii(const std::string &body) {
    uint8_t checksum = 0;
    for (const auto byte : body) {
        checksum ^= byte;
    }
    char suffix[6];
    std::snprintf(suffix, sizeof(suffix), "*%02X\r\n", checksum);
    return "$" + body + suffix;
}

struct Vn100Device {
    HardwareSerial port;
    std::vector<std::string> commands;
    std::string dropped_command;

    Vn100Device() {
        port.on_write = [this](const uint8_t *bytes, size_t size) {
            const std::string request(reinterpret_cast<const char *>(bytes), size);
            const std::string body = request.substr(1, request.find('*') - 1);
            commands.push_back(body);
            if (body == dropped_command) {
                return; // Simulate a device that never acknowledges this command.
            }
            const auto reply = vn100_ascii(body == "VNRRG,1" ? "VNRRG,1,VN-100" : body);
            port.rx.insert(port.rx.end(), reply.begin(), reply.end());
        };
    }

    void feed(const Packet &packet) {
        port.rx.insert(port.rx.end(), packet.begin(), packet.end());
    }
};

void append_le(Packet &packet, uint64_t value, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        packet.push_back(static_cast<uint8_t>(value >> (8 * i)));
    }
}

void append_floats(Packet &packet, std::initializer_list<float> values) {
    for (float value : values) {
        append_le(packet, std::bit_cast<uint32_t>(value), 4);
    }
}

Packet with_crc(Packet packet) {
    uint16_t crc = 0;
    for (size_t i = 1; i < packet.size(); ++i) {
        crc ^= static_cast<uint16_t>(packet[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    packet.push_back(crc >> 8);
    packet.push_back(crc & 0xff);
    return packet;
}

Packet imu_packet(uint64_t timestamp = 0x0102030405060708ULL) {
    // TimeStartup + Accel, AngularRate, SensSat (group masks 0x0001, 0x0E00).
    Packet packet{0xfa, 0x06, 0x01, 0x00, 0x00, 0x0e};
    append_le(packet, timestamp, 8);
    append_floats(packet, {1.25f, -2.5f, 9.75f, -0.125f, 0.25f, -0.5f});
    append_le(packet, 0x0255, 2); // Alternating saturation bits, including pressure.
    return with_crc(packet);
}

Packet attitude_packet() {
    // TimeStartup + Temperature, Pressure, Mag + YPR, Quaternion, LinAccelNed.
    Packet packet{0xfa, 0x16, 0x01, 0x00, 0x30, 0x01, 0x86, 0x00};
    append_le(packet, 0x1122334455667788ULL, 8);
    append_floats(packet, {24.5f, 101.25f, 0.125f, -0.25f, 0.5f,
                          135.0f, -22.5f, 45.0f, 0.5f, -0.5f, 0.5f, -0.5f,
                          1.5f, -2.25f, 3.75f});
    return with_crc(packet);
}

void assert_imu_sample(const vn100 &sensor, uint64_t timestamp = 0x0102030405060708ULL) {
    const auto &data = sensor.get_data();
    TEST_ASSERT_TRUE(data.imu_valid);
    TEST_ASSERT_EQUAL_UINT64(timestamp, data.imu_time.time_startup);
    TEST_ASSERT_EQUAL_FLOAT(1.25f, sensor.get_accel_X());
    TEST_ASSERT_EQUAL_FLOAT(-2.5f, sensor.get_accel_Y());
    TEST_ASSERT_EQUAL_FLOAT(9.75f, sensor.get_accel_Z());
    TEST_ASSERT_EQUAL_FLOAT(-0.125f, sensor.get_gyro_X());
    TEST_ASSERT_EQUAL_FLOAT(0.25f, sensor.get_gyro_Y());
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, sensor.get_gyro_Z());
    const auto &sat = data.saturation;
    TEST_ASSERT_TRUE(sat.mag_x);
    TEST_ASSERT_FALSE(sat.mag_y);
    TEST_ASSERT_TRUE(sat.mag_z);
    TEST_ASSERT_FALSE(sat.acc_x);
    TEST_ASSERT_TRUE(sat.acc_y);
    TEST_ASSERT_FALSE(sat.acc_z);
    TEST_ASSERT_TRUE(sat.gyro_x);
    TEST_ASSERT_FALSE(sat.gyro_y);
    TEST_ASSERT_FALSE(sat.gyro_z);
    TEST_ASSERT_TRUE(sat.pres);
}

void assert_attitude_sample(const vn100 &sensor) {
    const auto &data = sensor.get_data();
    TEST_ASSERT_TRUE(data.attitude_valid);
    TEST_ASSERT_EQUAL_UINT64(0x1122334455667788ULL, data.attitude_time.time_startup);
    TEST_ASSERT_EQUAL_FLOAT(24.5f, sensor.get_temperature());
    TEST_ASSERT_EQUAL_FLOAT(101.25f, data.pressure.pressure);
    const float mag[] = {0.125f, -0.25f, 0.5f};
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(mag, data.mag.mag, 3);
    TEST_ASSERT_EQUAL_FLOAT(135.0f, sensor.get_yaw());
    TEST_ASSERT_EQUAL_FLOAT(-22.5f, sensor.get_pitch());
    TEST_ASSERT_EQUAL_FLOAT(45.0f, sensor.get_roll());
    const float quaternion[] = {0.5f, -0.5f, 0.5f, -0.5f};
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(quaternion, data.quaternion.quaternion, 4);
    const float acceleration[] = {1.5f, -2.25f, 3.75f};
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(acceleration, data.lin_accel_ned.lin_accel_ned, 3);
}
} // namespace

void test_vn100_init_configures_binary_outputs_once() {
    Vn100Device device;
    vn100 sensor(device.port);
    TEST_ASSERT_FALSE(sensor.is_initialized());
    TEST_ASSERT_EQUAL_UINT32(0, device.port.baud);
    sensor.read();
    TEST_ASSERT_TRUE(device.commands.empty());

    sensor.init();
    TEST_ASSERT_TRUE(sensor.is_initialized());
    TEST_ASSERT_EQUAL_UINT32(921600, device.port.baud);
    TEST_ASSERT_FALSE(sensor.get_data().imu_valid);
    TEST_ASSERT_FALSE(sensor.get_data().attitude_valid);
    const char *expected[] = {"VNRRG,1", "VNASY,0", "VNWRG,6,0", "VNWRG,7,0",
                             "VNWRG,75,2,2,06,0001,0E00",
                             "VNWRG,76,2,8,16,0001,0130,0086",
                             "VNWRG,77,0,0,00", "VNASY,1"};
    TEST_ASSERT_EQUAL_UINT32(8, device.commands.size());
    for (size_t i = 0; i < device.commands.size(); ++i) {
        TEST_ASSERT_EQUAL_STRING(expected[i], device.commands[i].c_str());
    }
    sensor.init();
    TEST_ASSERT_EQUAL_UINT32(8, device.commands.size());
}

void test_vn100_simulated_init_failure_can_retry() {
    Vn100Device device;
    vn100 sensor(device.port);
    device.dropped_command = "VNWRG,76,2,8,16,0001,0130,0086";
    // The fake UART deliberately withholds a reply and micros() advances fake
    // time. Silence the expected driver logs during this software-only failure.
    const bool previous_console_output = Serial.console_output_enabled;
    Serial.console_output_enabled = false;
    sensor.init();
    Serial.console_output_enabled = previous_console_output;
    TEST_ASSERT_FALSE(sensor.is_initialized());
    TEST_ASSERT_FALSE(sensor.get_data().imu_valid);
    TEST_ASSERT_FALSE(sensor.get_data().attitude_valid);
    TEST_ASSERT_EQUAL_UINT32(6, device.commands.size());
    device.feed(imu_packet());
    sensor.read();
    TEST_ASSERT_FALSE(sensor.get_data().imu_valid);
    TEST_ASSERT_EQUAL_UINT32(imu_packet().size(), device.port.available());
    device.port.rx.clear();

    device.dropped_command.clear();
    sensor.init();
    TEST_ASSERT_TRUE(sensor.is_initialized());
    device.feed(imu_packet());
    sensor.read();
    assert_imu_sample(sensor);
}

void test_vn100_imu_packet_waits_for_complete_frame() {
    Vn100Device device;
    vn100 sensor(device.port);
    sensor.init();
    TEST_ASSERT_TRUE(sensor.is_initialized());
    const auto packet = imu_packet();
    device.feed(Packet(packet.begin(), packet.end() - 1));
    sensor.read();
    TEST_ASSERT_FALSE(sensor.get_data().imu_valid);
    TEST_ASSERT_EQUAL_UINT64(0, sensor.get_data().imu_time.time_startup);
    TEST_ASSERT_EQUAL_FLOAT(0, sensor.get_accel_X());
    device.feed(Packet{packet.back()});
    sensor.read();
    assert_imu_sample(sensor);
    TEST_ASSERT_FALSE(sensor.get_data().attitude_valid);
    sensor.read(); // No new bytes must retain the previous sample.
    assert_imu_sample(sensor);
}

void test_vn100_attitude_and_imu_updates_preserve_each_other() {
    Vn100Device device;
    vn100 sensor(device.port);
    sensor.init();
    TEST_ASSERT_TRUE(sensor.is_initialized());
    device.feed(attitude_packet());
    sensor.read();
    assert_attitude_sample(sensor);
    TEST_ASSERT_FALSE(sensor.get_data().imu_valid);
    device.feed(imu_packet());
    sensor.read();
    assert_imu_sample(sensor);
    assert_attitude_sample(sensor);
    device.feed(attitude_packet());
    sensor.read();
    assert_attitude_sample(sensor);
    assert_imu_sample(sensor);
}

void test_vn100_bad_packets_preserve_samples_and_recover() {
    Vn100Device device;
    vn100 sensor(device.port);
    sensor.init();
    TEST_ASSERT_TRUE(sensor.is_initialized());
    device.feed(imu_packet());
    device.feed(attitude_packet());
    sensor.read();
    assert_imu_sample(sensor);
    assert_attitude_sample(sensor);

    auto corrupt = imu_packet(99);
    corrupt[14] ^= 0x01; // Change payload without updating its checksum.
    device.feed(corrupt);
    sensor.read();
    assert_imu_sample(sensor);
    assert_attitude_sample(sensor);

    // A valid CRC does not make an unconfigured TimeStartup-only packet acceptable.
    Packet unexpected{0xfa, 0x02, 0x01, 0x00};
    append_le(unexpected, 99, 8);
    device.feed(with_crc(unexpected));
    sensor.read();
    assert_imu_sample(sensor);
    assert_attitude_sample(sensor);

    device.feed(imu_packet(123456789));
    sensor.read();
    assert_imu_sample(sensor, 123456789);
    assert_attitude_sample(sensor);
}
#endif

void setup() {
    delay(2000);
    SPI.begin();

    UNITY_BEGIN();

    RUN_TEST(test_sensor_read_called);
    RUN_TEST(test_sensor_init_called);
    RUN_TEST(test_send_to_comms_called);
    RUN_TEST(test_sensor_life_cycle);

    RUN_TEST(test_yaw_encoder_init);
    RUN_TEST(test_pitch_encoder_init);
    RUN_TEST(test_feeder_encoder_init);
    //RUN_TEST(test_yaw_encoder_read_returns_finite_float);
    RUN_TEST(test_pitch_encoder_read_returns_finite_float);

#ifdef UNIT_TEST
    RUN_TEST(test_vn100_init_configures_binary_outputs_once);
    RUN_TEST(test_vn100_simulated_init_failure_can_retry);
    RUN_TEST(test_vn100_imu_packet_waits_for_complete_frame);
    RUN_TEST(test_vn100_attitude_and_imu_updates_preserve_each_other);
    RUN_TEST(test_vn100_bad_packets_preserve_samples_and_recover);
#endif

    UNITY_END();
}

void loop() {}
