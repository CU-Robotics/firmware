  /// Questions:
  /// How often does i2C communicate between the Teensy and the IMU? (SCL: Clock pin. SDA: data pin)
  /// Can we optimize adafruit behavior, or is it hardware limited?
  /// Can we put the IMU ionto bootloader mode to change firmware? We might be able to change the clock speed to be faster (if it is hardcoded).
      /// No information gained from bottom of IMU. But it has relevant hardware specs/capabilities on it.


// #include <Arduino.h> // https://www.arduino.cc/reference/en/
#include <Wire.h> // https://www.arduino.cc/reference/en/language/functions/communication/wire/
// learning about wire communication: https://docs.arduino.cc/learn/communication/wire

void setup() {
  Wire.begin(0x6A);
  Serial.begin(115200);

  Serial.println("Setup!");

  
}

void loop() {
  // Serial.println("loop");

  // lsm6dosx I2C addresses: 0x1C, 0x6A
  Wire.requestFrom(0x6A, 7); 
  // char c = Wire.read();

  // Serial.printf("%d", c);



  Serial.println(Wire.available());
  while(Wire.available()) {
        Serial.println("aaaaa");
        char c = Wire.read();    // Receive a byte as character
        Serial.print(c);         // Print the character
  }

  // Serial.println("after");
  // Serial.println(avail);
  // Wire.onReceive([](int) -> void { 
  //     Serial.print("aaaaa");
  //     char c = Wire.read();
  //     Serial.print(c);
  // });

  delay(500);
}
