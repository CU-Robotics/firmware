#include <Adafruit_BMP280.h>
#include <BMI160Gen.h>
#include <CurieIMU.h>

#define Arduino_UNO
#define I2C

BMI160GenClass BMI160;
const int select_pin = 10;
Adafruit_BMP280 BMP280;

void setup() {
  Serial.begin(9600); // initialize Serial communication
  Serial.println("Programme de Test Capteur BMI+BMM");
  while (!Serial);    // wait for the serial port to open

  // initialize device
  BMI160.begin(select_pin);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

  BMI160.setGyroRange(250);
  BMI160.setAccelerometerRange(4);
}

void loop() {
  uint8_t IF_CONF = 107;
  uint8_t FIFO_CONF = 71;
  uint8_t MAG_IF_0 = 75;
  uint8_t MAG_IF_1 = 76;
  uint8_t MAG_IF_2 = 77;
  uint8_t MAG_IF_3 = 78;
  uint8_t MAG_IF_4 = 79;
  uint8_t DATA_T_L = 32;
  uint8_t DATA_T_M = 33;
  uint8_t DATA_MAG_X_L = 4;
  uint8_t DATA_MAG_X_M = 5;
  uint8_t MAG_CONF = 68;
  uint8_t FIFO_L = 70;
  uint8_t FIFO_M = 71;
  uint8_t STATUS = 27;
  uint8_t Res_IF_CONF, Res_MAG_IF_0, Res_MAG_IF_1, Res_MAG_IF_2, Res_MAG_IF_3, Res_MAG_IF_4, Res_Data_t_l, Res_Data_t_m, Res_STATUS, Res_DATA_MAG_X_L, Res_DATA_MAG_X_M, Res_MAG_CONF, Res_FIFO_CONF;
  uint8_t I2C_MAG = 32;
  uint8_t MAG_MAN_DIS = 3;
  uint8_t MAG_MAN_8BitsVal = 128;
  uint8_t ADD_BMM_DATA = 66;
  int mx, my, mz, gx, gy, gz;
  float MagRate;

  MagRate = BMI160.getMagRate();
  BMI160.setRegister(MAG_IF_0, I2C_MAG);
  BMI160.setRegister(MAG_IF_1,MAG_MAN_8BitsVal);
  BMI160.setRegister(MAG_IF_2, ADD_BMM_DATA);
  BMI160.setRegister(MAG_IF_3, 78);

  delay(250);
  
  Res_IF_CONF = BMI160.getRegister(IF_CONF);
  Res_FIFO_CONF = BMI160.getRegister(FIFO_CONF);
  Res_MAG_IF_0 = BMI160.getRegister(MAG_IF_0);
  Res_MAG_IF_1 = BMI160.getRegister(MAG_IF_1);
  Res_MAG_IF_2 = BMI160.getRegister(MAG_IF_2);
  Res_MAG_IF_3 = BMI160.getRegister(MAG_IF_3);
  Res_MAG_IF_4 = BMI160.getRegister(MAG_IF_4);

  Res_Data_t_l = BMI160.getRegister(DATA_T_L);
  Res_Data_t_m = BMI160.getRegister(DATA_T_M);

  //Res_DATA_MAG_X_L = BMI160.getRegister(DATA_MAG_X_L);
  //Res_DATA_MAG_X_M = BMI160.getRegister(DATA_MAG_X_M);

  Res_STATUS = BMI160.getRegister(STATUS);
  Res_MAG_CONF = BMI160.getRegister(MAG_CONF);

  BMI160.readMag(mx, my, mz);
  BMI160.readGyro(gx, gy, gz);

  Serial.print("Valeur du registre IF_CONF :\t");
  Serial.print(int(Res_IF_CONF));
  Serial.println();
  Serial.print("data à ecrire :\t");
  Serial.print((Res_MAG_IF_4));
  Serial.println();
  //Serial.print(int(Res_DATA_MAG_X_M));
  //Serial.print(int(Res_DATA_MAG_X_L));
  Serial.println();
  Serial.print("Freq echantillonage Magnetometre :\t");
  Serial.print(MagRate);
  Serial.println();
  Serial.print("Valeur du registre DATA pour le mag :\t");
  Serial.print(mx);
  Serial.print("\t");
  Serial.print(my);
  Serial.print("\t");
  Serial.print(mz);
  Serial.println();
  Serial.print("Valeur du registre DATA pour le Gyro :\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.println();
  Serial.print("Valeur du registre status: \t");
  Serial.print(Res_STATUS);
  Serial.println();
  Serial.print("Valeur du registre Mag_conf: \t");
  Serial.print(Res_MAG_CONF);
  Serial.print("Valeur du registre FIFO_conf: \t");
  Serial.print(Res_FIFO_CONF);
  Serial.println();
  Serial.println();

  delay(500);
}


