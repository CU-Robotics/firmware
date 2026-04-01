#include <Arduino.h>
#include <unity.h>

#include "sensors/Sensor.hpp"

class DummySensor : public Sensor {
  public:
    explicit DummySensor(uint8_t id) : Sensor(BUFFENC, id) {}

    void read() override {read_called_ = true;}
    void init() override {init_called_ = true;}
    void sendToComms() const override {send_called_ = true;}

    bool wasReadCalled() const {return read_called_;}
    bool wasInitCalled() const {return init_called_;}
    bool wasSend2ComsCalled() const {return send_called_;}

  private:
    bool read_called_ = false;
    bool init_called_ = false;
    mutable bool send_called_  = false;
};

void testSensorReadCalled() {
    DummySensor sensor;
    sensor.read();
    TEST_ASSERT_TRUE(sensor.wasReadCalled());
}

void testSensorInitCalled() {
    DummySensor sensor;
    sensor.init();
    TEST_ASSERT_TRUE(sensor.wasInitCalled());
}

void testSend2Coms() {
    DummySensor sensor;
    sensor.sendToComms();
    TEST_ASSERT_TRUE(sensor.wasSend2ComsCalled());
}

void testSensorLifeCycle() {
    DummySensor sensor;
    sensor.init();
    sensor.read();
    sensor.send_to_comms();

    TEST_ASSERT_TRUE(sensor.wasInitCalled());
    TEST_ASSERT_TRUE(sensor.wasReadCalled());
    TEST_ASSERT_TRUE(sensor.wasSendCalled());
}

void setup() {
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(testSensorReadCalled);
    RUN_TEST(testSensorInitCalled);
    RUN_TEST(testSend2Coms);
    RUN_TEST(testSensorLifeCycle);
    UNITY_END();
}

void loop() {}
