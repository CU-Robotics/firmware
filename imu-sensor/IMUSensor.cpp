#include <Arduino.h> // https://www.arduino.cc/reference/en/
#include <Wire.h> // https://www.arduino.cc/reference/en/language/functions/communication/wire/

#include <IMUSensor.hpp>

IMUSensor::IMUSensor() {
    // initialize IMU
    Wire.read()
    Serial.begin(115200);
}

void IMUSensor::read() {
    // I2C on SDA and SCL pins.
    // Serial.digitalRead(pin)

}