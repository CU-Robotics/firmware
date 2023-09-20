#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#include <imxrt_flexcan.h>
#include <isotp.h>
#include <isotp_server.h>
#include <kinetis_flexcan.h>
//#include "C:\Users\lachl\OneDrive\Desktop\CURobotics\firmware/read_write.hpp"

//FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;

class ReadWrite {
private:
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

    int output[2][16];
    int input[2][8][8];


    struct Motor {
      int mtrNum;
      int theta;
      int rotSpd;
      int trq;
      uint16_t tmp;
    }; Motor motors[2][8];


public:
    ReadWrite() {

    }
    

    void init() {
        can1.begin();
        can1.setBaudRate(1000000);
        can1.enableFIFO();

        can2.begin();
        can2.setBaudRate(1000000);
        can2.enableFIFO();
    }

    int getAngle(int motID, int canNum);
    int getRotSpd(int motID, int canNum);
    int getTrqCur(int motID, int canNum);

    void readCAN() {
      CAN_message_t msg;

      while (can1.read(msg)) {

        int index = msg.id - 0x201;

        motors[0][index].theta = (msg.buf[0] << 8) | msg.buf[1];

        if (index == 3) {
          union Conv {
            struct {
              uint16_t one;
              uint16_t two;
            } Byte;
            int val;
          }; Conv conv;
          conv.Byte.one = msg.buf[0];
          conv.Byte.two = msg.buf[1];

          

          Serial.print("Read: ");
          

          Serial.print(msg.buf[0]);
          Serial.print(" : ");
          Serial.print(msg.buf[1]);
          Serial.print("   ===   ");
          Serial.print(motors[0][index].theta);
          Serial.print("   ==   ");
          Serial.print(conv.val);
          Serial.println();
        }
        motors[0][index].rotSpd = (msg.buf[2] << 8) | msg.buf[3];
        motors[0][index].trq = (msg.buf[4] << 8) | msg.buf[5];
        motors[0][index].tmp = msg.buf[6];
      }

      while (can2.read(msg)) {

        int index = msg.id - 0x200;

        motors[1][index].theta = (msg.buf[0] << 8) | msg.buf[1];
        motors[1][index].rotSpd = (msg.buf[2] << 8) | msg.buf[3];
        motors[1][index].trq = (msg.buf[4] << 8) | msg.buf[5];
        motors[1][index].tmp = msg.buf[6];
      }
    }
    
    void writeCAN() {
          CAN_message_t out;
          out.id = 0x200;
          for (int val = 0; val < 8; val++) {
              out.buf[val] = output[0][val];
          }
          can1.write(out);

          out.id = 0x1FF;
          for (int val = 8; val < 16; val++) {
              out.buf[val-8] = output[0][val];
          }
          can1.write(out);

          out.id = 0x200;
          for (int val = 0; val < 8; val++) {
              out.buf[0] = output[1][val];
          }
          can2.write(out);

          out.id = 0x1FF;
          for (int val = 8; val < 16; val++) {
              out.buf[val-8] = output[1][val];
          }
          can2.write(out);
    }


    void printCAN(int mtrID, int canID) {
      Serial.print("CAN #");
      Serial.println(mtrID);
      Serial.print("{\t");


      // for (int i = 0; i < 8; i++) {
      //     Serial.print(input[canID][mtrID-1][i]);
      //     Serial.print("\t");
      // // }

      Serial.print("Speed: ");
      Serial.print(motors[canID-1][mtrID-1].rotSpd);
      Serial.print("\t");

      Serial.print("Angle: ");
      Serial.print(motors[canID-1][mtrID-1].theta);
      Serial.print("\t");

      Serial.print("Torque: ");
      Serial.print(motors[canID-1][mtrID-1].trq);
      Serial.print("\t");

      Serial.print("Temp: ");
      Serial.print(motors[canID-1][mtrID-1].tmp);
      Serial.print("\t");

      Serial.print("}");
      Serial.println();
    }

    void zeroCAN() {
      CAN_message_t out;
      out.id = 0x200;
      for (int val = 0; val < 8; val++) {
          out.buf[val] = 0;
      }
      can1.write(out);

      out.id = 0x1FF;
      for (int val = 8; val < 16; val++) {
          out.buf[val-8] = 0;
      }
      can1.write(out);

      out.id = 0x200;
      for (int val = 0; val < 8; val++) {
          out.buf[0] = 0;
      }
      can2.write(out);

      out.id = 0x1FF;
      for (int val = 8; val < 16; val++) {
          out.buf[val-8] = 0;
      }
      can2.write(out);
    }


    void writeMtr(short val, int motID, int canNum) {
      output[canNum][motID*2-2] = (val >> 8) & 0xff;
      output[canNum][motID*2-1] = val & 0xff;
    }

};

ReadWrite readWriteObj;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  readWriteObj.init();
  readWriteObj.zeroCAN();

}

void loop() {

  readWriteObj.readCAN();
  //readWriteObj.printCAN(4, 1);

  // readWriteObj.writeMtr(0, 2, 0);
  // readWriteObj.writeCAN();

}