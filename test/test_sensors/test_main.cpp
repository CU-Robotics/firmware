#include <Arduino.h>
#include <unity.h>

#include "sensors/sensor.hpp"

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

void setup() {
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(test_sensor_read_called);
    RUN_TEST(test_sensor_init_called);
    RUN_TEST(test_send_to_comms_called);
    RUN_TEST(test_sensor_life_cycle);
    UNITY_END();
}

void loop() {}