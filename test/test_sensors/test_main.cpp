#include <Arduino.h>
#include <unity.h>

#include "sensors/buff_encoder.hpp"

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
    RUN_TEST(test_yaw_encoder_read_returns_finite_float);
    RUN_TEST(test_pitch_encoder_read_returns_finite_float);

    UNITY_END();
}

void loop() {}