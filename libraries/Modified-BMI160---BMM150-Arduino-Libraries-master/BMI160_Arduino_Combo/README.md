# Arduino_BMI160_BMM150
An Arduino Library for the BMI160, a 6-Axis Sensor from BOSCH

This driver is connected through the Arduino SPI.
This library is derived from the Intel's CurieIMU driver for the Arduino/Genuino 101 and from the work of Hanyazou.

Intel's driver repository: https://github.com/01org/corelibs-arduino101/tree/master/libraries/CurieIMU

Hanyazou's BMI160-Arduino repository: https://github.com/hanyazou/BMI160-Arduino

BMI160: https://www.bosch-sensortec.com/bst/products/all_products/bmi160

## How to install
Copy all files of this project to the your Arduino IDE library folder.

## Circuit
You should connect some digital out pin to the CSB of the BMI160 and tell the number of the pin to the initialize method, begin().