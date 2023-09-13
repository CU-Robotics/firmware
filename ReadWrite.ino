#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#include <imxrt_flexcan.h>
#include <isotp.h>
#include <isotp_server.h>
#include <kinetis_flexcan.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;


// Begin new code (not tested)
int setMotor(int motorID, short value);

int setMotor(int motorID, short value) {
  CAN_message_t out;

  union stob {
    short num;
    struct Byte {
      char c1,c2;
    }
  } stob converter;
  converter.num = value;

  out.id = motorID;

  out.buf[motorID * 2 - 2] = converter.c2;
  out.buf[motorID * 2 - 1] = converter.c1;

  return can.write(out);
}

// Begin old code (has been tested and works)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //Initialise the CAN bus
  can.begin();
  can.setBaudRate(1000000);
  can.enableFIFO();

}

void loop() {

  CAN_message_t out;
  out.id = 0x200;
  out.buf[0] = 0;
  out.buf[1] = 0;
  out.buf[2] = 0;
  out.buf[3] = 0;
  out.buf[4] = 0;
  out.buf[5] = 0;
  out.buf[6] = 0;
  out.buf[7] = 0;
  
  int outSuc = can.write(out);

  CAN_message_t msg;

  while (can.read(msg)) {


    if (msg.id != 0x204) continue;
    Serial.println("\t======");
    Serial.print("\tid: "); Serial.println(msg.id, HEX);
    Serial.print("\t\t[");
    Serial.print(msg.buf[0]);
    Serial.print("\t");
    Serial.print(msg.buf[1]);

    Serial.print("\t");
    Serial.print(msg.buf[2]);
    Serial.print("\t");
    Serial.print(msg.buf[3]);
    Serial.print("\t");
    Serial.print(msg.buf[4]);
    Serial.print("\t");
    Serial.print(msg.buf[5]);
    Serial.print("\t");
    Serial.print(msg.buf[6]);
    Serial.print("\t");
    Serial.print(msg.buf[7]);  
    Serial.println("]");
  }

}