#include <Arduino.h>
#include <unity.h>
 
#include "sensors/sensor_manager.hpp"
#include "sensors/buff_encoder.hpp"
#include "sensors/ICM20649.hpp"
#include "sensors/StereoCamTrigger.hpp"
 
// ─────────────────────────────────────────────
// DummySensor — original tests, untouched
// ─────────────────────────────────────────────
 
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
 
// ─────────────────────────────────────────────
// Gerald configs
// ─────────────────────────────────────────────
 
// --- Buff Encoders ---
// Shared SPI bus: MISO=12, MOSI=11, CLK=13
static Cfg::BuffEncoder make_yaw_encoder_cfg() {
    Cfg::BuffEncoder cfg{};
    cfg.spi_chip_select_pin = 37;
    cfg.spi_miso_pin        = 12;
    cfg.spi_mosi_pin        = 11;
    cfg.spi_clock_pin       = 13;
    cfg.encoder_name        = Cfg::SensorName::YawEncoder;
    return cfg;
}
 
static Cfg::BuffEncoder make_pitch_encoder_cfg() {
    Cfg::BuffEncoder cfg{};
    cfg.spi_chip_select_pin = 28;
    cfg.spi_miso_pin        = 12;
    cfg.spi_mosi_pin        = 11;
    cfg.spi_clock_pin       = 13;
    cfg.encoder_name        = Cfg::SensorName::PitchEncoder;
    return cfg;
}
 
static Cfg::BuffEncoder make_lower_feeder_encoder_cfg() {
    Cfg::BuffEncoder cfg{};
    cfg.spi_chip_select_pin = 9;
    cfg.spi_miso_pin        = 12;
    cfg.spi_mosi_pin        = 11;
    cfg.spi_clock_pin       = 13;
    cfg.encoder_name        = Cfg::SensorName::LowerFeederEncoder;
    return cfg;
}
 
static Cfg::BuffEncoder make_upper_feeder_encoder_cfg() {
    Cfg::BuffEncoder cfg{};
    cfg.spi_chip_select_pin = 29;
    cfg.spi_miso_pin        = 12;
    cfg.spi_mosi_pin        = 11;
    cfg.spi_clock_pin       = 13;
    cfg.encoder_name        = Cfg::SensorName::UpperFeederEncoder;
    return cfg;
}
 
// --- ICM IMU ---
// Yaw IMU: separate SPI bus — MISO=39, MOSI=26, CLK=27, CS=10
static Cfg::IcmImu make_yaw_imu_cfg() {
    Cfg::IcmImu cfg{};
    cfg.communication_protocol = static_cast<uint32_t>(Cfg::CommunicationProtocol::SPI);
    cfg.spi_chip_select_pin    = 10;
    cfg.spi_miso_pin           = 39;
    cfg.spi_mosi_pin           = 26;
    cfg.spi_clock_pin          = 27;
    cfg.num_calibration_reads  = 500;
    cfg.accel_range            = static_cast<uint32_t>(Cfg::ICMAccelRange::Accel30G);
    cfg.gyro_rate_range        = static_cast<uint32_t>(Cfg::ICMGyroRateRange::DPS4000);
    cfg.imu_name               = Cfg::SensorName::YawIcmImu;
    return cfg;
}
 
// --- Stereo Camera Trigger ---
static Cfg::StereoCameraTrigger make_stereo_trigger_cfg() {
    Cfg::StereoCameraTrigger cfg{};
    cfg.digital_trigger_pin_1  = 33;
    cfg.digital_trigger_pin_2  = 21;
    cfg.fps                    = 155;
    cfg.trigger_pulse_width_us = 14;
    cfg.trigger_name           = Cfg::SensorName::StereoCameraTrigger;
    return cfg;
}
 
// ─────────────────────────────────────────────
// BuffEncoder tests
// ─────────────────────────────────────────────
 
void test_yaw_encoder_init() {
    auto cfg = make_yaw_encoder_cfg();
    BuffEncoder encoder(cfg);
    encoder.init();
    // If init completes without triggering safety, pins and SPI are configured correctly
    TEST_ASSERT_EQUAL(Cfg::SensorName::YawEncoder, encoder.get_name());
}
 
void test_pitch_encoder_init() {
    auto cfg = make_pitch_encoder_cfg();
    BuffEncoder encoder(cfg);
    encoder.init();
    TEST_ASSERT_EQUAL(Cfg::SensorName::PitchEncoder, encoder.get_name());
}
 
void test_lower_feeder_encoder_init() {
    auto cfg = make_lower_feeder_encoder_cfg();
    BuffEncoder encoder(cfg);
    encoder.init();
    TEST_ASSERT_EQUAL(Cfg::SensorName::LowerFeederEncoder, encoder.get_name());
}
 
void test_upper_feeder_encoder_init() {
    auto cfg = make_upper_feeder_encoder_cfg();
    BuffEncoder encoder(cfg);
    encoder.init();
    TEST_ASSERT_EQUAL(Cfg::SensorName::UpperFeederEncoder, encoder.get_name());
}
 
void test_yaw_encoder_read_returns_float() {
    auto cfg = make_yaw_encoder_cfg();
    BuffEncoder encoder(cfg);
    encoder.init();
    encoder.read();
    // Angle should be a finite float — NaN or inf would indicate a bad SPI read
    float angle = encoder.get_angle();
    TEST_ASSERT_FALSE(isnan(angle));
    TEST_ASSERT_FALSE(isinf(angle));
}
 
void test_pitch_encoder_read_returns_float() {
    auto cfg = make_pitch_encoder_cfg();
    BuffEncoder encoder(cfg);
    encoder.init();
    encoder.read();
    float angle = encoder.get_angle();
    TEST_ASSERT_FALSE(isnan(angle));
    TEST_ASSERT_FALSE(isinf(angle));
}
 
void test_buff_encoder_send_to_comms_does_not_crash() {
    auto cfg = make_yaw_encoder_cfg();
    BuffEncoder encoder(cfg);
    encoder.init();
    encoder.read();
    encoder.send_to_comms();
    TEST_PASS();
}
 
// ─────────────────────────────────────────────
// ICM IMU tests
// ─────────────────────────────────────────────
 
void test_yaw_imu_init() {
    auto cfg = make_yaw_imu_cfg();
    ICM20649 imu(cfg);
    imu.init();
    TEST_PASS(); // passes if SPI setup and chip detection succeed without safety trip
}
 
void test_yaw_imu_read_does_not_crash() {
    auto cfg = make_yaw_imu_cfg();
    ICM20649 imu(cfg);
    imu.init();
    imu.read();
    TEST_PASS();
}
 
void test_yaw_imu_send_to_comms_does_not_crash() {
    auto cfg = make_yaw_imu_cfg();
    ICM20649 imu(cfg);
    imu.init();
    imu.read();
    imu.send_to_comms();
    TEST_PASS();
}
 
// ─────────────────────────────────────────────
// Stereo Camera Trigger tests
// ─────────────────────────────────────────────
 
void test_stereo_trigger_init() {
    auto cfg = make_stereo_trigger_cfg();
    StereoCamTrigger trigger(cfg);
    trigger.init();
    TEST_PASS();
}
 
void test_stereo_trigger_read_does_not_crash() {
    auto cfg = make_stereo_trigger_cfg();
    StereoCamTrigger trigger(cfg);
    trigger.init();
    trigger.read();
    TEST_PASS();
}
 
void test_stereo_trigger_send_to_comms_does_not_crash() {
    auto cfg = make_stereo_trigger_cfg();
    StereoCamTrigger trigger(cfg);
    trigger.init();
    trigger.read();
    trigger.send_to_comms();
    TEST_PASS();
}
 
// ─────────────────────────────────────────────
// setup / loop
// ─────────────────────────────────────────────
 
void setup() {
    delay(2000);
    SPI.begin();
 
    UNITY_BEGIN();
 
    // ── Original DummySensor tests ──
    RUN_TEST(test_sensor_read_called);
    RUN_TEST(test_sensor_init_called);
    RUN_TEST(test_send_to_comms_called);
    RUN_TEST(test_sensor_life_cycle);
 
    // ── BuffEncoder (Gerald pins) ──
    RUN_TEST(test_yaw_encoder_init);
    RUN_TEST(test_pitch_encoder_init);
    RUN_TEST(test_lower_feeder_encoder_init);
    RUN_TEST(test_upper_feeder_encoder_init);
    RUN_TEST(test_yaw_encoder_read_returns_float);
    RUN_TEST(test_pitch_encoder_read_returns_float);
    RUN_TEST(test_buff_encoder_send_to_comms_does_not_crash);
 
    // ── ICM20649 Yaw IMU (Gerald pins) ──
    RUN_TEST(test_yaw_imu_init);
    RUN_TEST(test_yaw_imu_read_does_not_crash);
    RUN_TEST(test_yaw_imu_send_to_comms_does_not_crash);
 
    // ── Stereo Camera Trigger ──
    RUN_TEST(test_stereo_trigger_init);
    RUN_TEST(test_stereo_trigger_read_does_not_crash);
    RUN_TEST(test_stereo_trigger_send_to_comms_does_not_crash);
 
    UNITY_END();
}
 
void loop() {}