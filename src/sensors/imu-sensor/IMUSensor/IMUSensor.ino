  /// Questions:
  /// How often does i2C communicate between the Teensy and the IMU? (SCL: Clock pin. SDA: data pin)
  /// Can we optimize adafruit behavior, or is it hardware limited?
  /// Can we put the IMU ionto bootloader mode to change firmware? We might be able to change the clock speed to be faster (if it is hardcoded).
      /// No information gained from bottom of IMU. But it has relevant hardware specs/capabilities on it.


// #include <Arduino.h> // https://www.arduino.cc/reference/en/
#include <Wire.h> // https://www.arduino.cc/reference/en/language/functions/communication/wire/
// learning about wire communication: https://docs.arduino.cc/learn/communication/wire

void setup() {
  Wire.begin(0x1C);
  Serial.begin(115200);

  Serial.println("Setup!");

  

  // guide on writing to registers: https://thecavepearlproject.org/2017/11/03/configuring-i2c-sensors-with-arduino/
  // Wire.beginTransmission(0x1C);  // Attention sensor @ deviceAddress!

  // XL_ULP_EN: CTRL5_C bit 7 
  // Wire.write(0x14h);   // command byte to target the register location
  // Wire.write(0);                           // new data to put into that memory register

  // // G_HM_MODE: CTRL7_G bit 7
  // Wire.write(0x16h);   // command byte to target the register location
  // Wire.write(0);                           // new data to put into that memory register



  // CTRL1_XL: 0x10h, ODR_XL3 ODR_XL2 ODR_XL1 ODR_XL0 (bits 7-4)
  // Wire.read(0x14h, )
  // Wire.endTransmission();


}

void loop() {
  // Serial.println("loop");
  // lsm6dosx I2C addresses: 0x1C, 0x6A
  Wire.requestFrom(2, 8); 
  // char c = Wire.read();

  // Serial.printf("%d", c);



  // Serial.println(Wire.available());

  // while(Wire.available()) {
        // Serial.println("aaaaa");
        char c = Wire.read();    // Receive a byte as character
        Serial.print(c);         // Print the character
  // }

  // Serial.println("after");
  // Serial.println(avail);
  // Wire.onReceive([](int) -> void { 
  //     Serial.print("aaaaa");
  //     char c = Wire.read();
  //     Serial.print(c);
  // });

  delay(500);
}
