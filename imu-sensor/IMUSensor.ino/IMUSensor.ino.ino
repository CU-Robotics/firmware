// #include <Arduino.h> // https://www.arduino.cc/reference/en/
#include <Wire.h> // https://www.arduino.cc/reference/en/language/functions/communication/wire/

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
