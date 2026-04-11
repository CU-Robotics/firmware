#include <Arduino.h>
#include <unity.h>

#include "sensors/buff_encoder.hpp"
#include "sensors/ICM20649.hpp"
#include "sensors/StereoCamTrigger.hpp"

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

// Gerald configs - The grim reaper of firmware wanted me to do with gerald should be pretty easy to change for other configs

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
    cfg.spi_cs = 9;   // Lower feeder encoder CS from gerald.yaml
    cfg.spi_miso = 12;
    cfg.spi_mosi = 11;
    cfg.spi_sck = 13;
    cfg.encoder_name = Cfg::SensorName::FeederBuffEncoder;
    return cfg;
}

// ICM Yaw IMU — separate SPI bus: MISO=39, MOSI=26, SCK=27, CS=10
static Cfg::IcmImu make_yaw_imu_cfg() {
    Cfg::IcmImu cfg{};
    cfg.communication_protocol = Cfg::CommunicationProtocol::SPI;
    cfg.spi_cs = 10;
    cfg.spi_miso = 39;
    cfg.spi_mosi = 26;
    cfg.spi_sck = 27;
    cfg.num_calibration_reads = 500;
    cfg.accel_range = Cfg::ICMImuAccelRange::A_30G;
    cfg.gyro_range = Cfg::ICMImuGyroRange::DPS4000;
    cfg.imu_name = Cfg::SensorName::YawIcmImu;
    return cfg;
}

// Stereo camera trigger: Pin1=33, Pin2=21, 155fps, 14µs pulse
static Cfg::StereoCamTrigger make_stereo_trigger_cfg() {
    Cfg::StereoCamTrigger cfg{};
    cfg.digital_trigger_pin_1 = 33;
    cfg.digital_trigger_pin_2 = 21;
    cfg.fps = 155;
    cfg.trigger_pulse_width = 14;
    cfg.camera_trigger_name = Cfg::SensorName::StereoCameraTrigger;
    return cfg;
}

// BuffEncoder tests

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

void test_yaw_encoder_read_returns_finite_float() {
    auto cfg = make_yaw_encoder_cfg();
    BuffEncoder encoder(cfg);
    encoder.init();
    encoder.read();
    float angle = encoder.get_angle();
    TEST_ASSERT_FALSE(isnan(angle));
    TEST_ASSERT_FALSE(isinf(angle));
}

void test_pitch_encoder_read_returns_finite_float() {
    auto cfg = make_pitch_encoder_cfg();
    BuffEncoder encoder(cfg);
    encoder.init();
    encoder.read();
    float angle = encoder.get_angle();
    TEST_ASSERT_FALSE(isnan(angle));
    TEST_ASSERT_FALSE(isinf(angle));
}

// ICM20649 Yaw IMU tests

void test_yaw_imu_init() {
    auto cfg = make_yaw_imu_cfg();
    ICM20649 imu(cfg);
    imu.init();
    TEST_PASS();
}

void test_yaw_imu_read_does_not_crash() {
    auto cfg = make_yaw_imu_cfg();
    ICM20649 imu(cfg);
    imu.init();
    imu.read();
    TEST_PASS();
}

// StereoCamTrigger tests

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

void setup() {
    delay(2000);
    SPI.begin();

    UNITY_BEGIN();

    // OG dummy sensor
    RUN_TEST(test_sensor_read_called);
    RUN_TEST(test_sensor_init_called);
    RUN_TEST(test_send_to_comms_called);
    RUN_TEST(test_sensor_life_cycle);

    // BuffEncoder with Gerald pins
    RUN_TEST(test_yaw_encoder_init);
    RUN_TEST(test_pitch_encoder_init);
    RUN_TEST(test_feeder_encoder_init);
    RUN_TEST(test_yaw_encoder_read_returns_finite_float);
    RUN_TEST(test_pitch_encoder_read_returns_finite_float);

    // ICM20649 Yaw IMU with Gerald pins
    RUN_TEST(test_yaw_imu_init);
    RUN_TEST(test_yaw_imu_read_does_not_crash);

    // StereoCamTrigger
    RUN_TEST(test_stereo_trigger_init);
    RUN_TEST(test_stereo_trigger_read_does_not_crash);

    UNITY_END();
}

void loop() {}