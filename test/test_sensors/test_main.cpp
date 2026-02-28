#include <Arduino.h>
#include <unity.h>

#include "sensors/Sensor.hpp"

class DummySensor : public Sensor {
  public:
    explicit DummySensor(uint8_t id) : Sensor(BUFFENC, id) {}

    bool read() override {
        read_called_ = true;
        return true;
    }

    bool wasReadCalled() const { return read_called_; }

  private:
    bool read_called_ = false;
};

void test_sensor_type_and_id() {
    DummySensor sensor(7);

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BUFFENC), static_cast<uint8_t>(sensor.getType()));
    TEST_ASSERT_EQUAL_UINT8(7, sensor.getId());

    sensor.setId(9);
    TEST_ASSERT_EQUAL_UINT8(9, sensor.getId());
}

void setup() {
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(test_sensor_type_and_id);
    UNITY_END();
}

void loop() {}
