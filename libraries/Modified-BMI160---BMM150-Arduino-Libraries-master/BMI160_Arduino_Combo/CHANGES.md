 Changes from the BMI160 LIbrary made by Hanyazou
==
Changes in BMI160.cpp and BMI160.h
-

* Added #define needed to access the magnetometer's data and configurate it  
* Added function getMotion9()  
* Modified the function initialize to add the magnetometer's power up  
* Added typedef enum BMI160MagRate & BMI160MagRange in the header  
* Added functions getMagRate() and setMagRate()  
* Added functions getMagneto(), getMagnetoX(), getMagnetoY() & getMagnetoZ()  
* Added functions getMagFIFOEnabled() & setMagFIFOEnabled()  

Changes in BMI160Gen.cpp and BMI160Gen.h
-

* Deleted line 15 in BMI160Gen.h (Bug fix)  
* Deleted line 27 in BMI160Gen.cpp (Bug fix)  

Changes in CurieIMU.cpp and CurieIMU.h
-

* Added readMotionSensor9()  
* Added SetMagRate() & getMagRate()  
* 
