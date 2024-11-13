# Modified-BMI160---BMM150-Arduino-Libraries
Update Arduino/Teensy Drivers for the BMM150 and BMI160 Bosch Sensors

This repository is a combination of three of existing libraries for the referenced Bosch sensors:
1. BMI160-Arduino - This is a direct copy of the Hanyazou's BMI160-Arduino repository: https://github.com/hanyazou/BMI160-Arduino.  No changes were made to this library.
2. Grove_3_Axis_Compass_V2.0_BMM150 - is a slighly modifed version of the SeedStudio respository (https://github.com/Seeed-Studio/Grove_3_Axis_Compass_V2.0_BMM150) which is modeled after the Bosch BMI150 API.  I added two additional functions to simplify modifications to the ODR and setup the DRDY interrupt if you choose to use the DRDY:
  - set_mag_rate(uint8_t rate) - where the uint8 rate is defined by the enum for the BMI ODR in the .h file
  - set_DRDY_bit(uint8_t enable_bit) - it can be either 1 or 0, 1 to use a interrup or 0 not too.  Or just not call it and the interrupt won't be set.

3. BMI160_Arduino_Combo - is an updated library for using the BMM150 in passthrough mode with a BMI160.  The original library can be found here: BMI160_Arduino_Combo.  The library has been reworked to reflect the changes in the BMI160 library so it can now operate in I2c or SPI modes.  NOTE: i have only tested this with I2C.

I have also included 3 sketches that illustrate the use of each library.

Enjoy.
