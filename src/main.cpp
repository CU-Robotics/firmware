#include <Arduino.h>

#include "utils/timing.hpp"
#include "comms/rm_can.hpp"
#include "sensors/dr16.hpp"
#include "filters/pid_filter.hpp"
#include "SPI.h"


#define MT6835_OP_READ  0b0011
#define MT6835_OP_WRITE 0b0110
#define MT6835_OP_PROG  0b1100
#define MT6835_OP_ZERO  0b0101
#define MT6835_OP_ANGLE 0b1010

#define MT6835_CMD_MASK  0b111100000000000000000000
#define MT6835_ADDR_MASK 0b000011111111111100000000
#define MT6835_DATA_MASK 0b000000000000000011111111

#define MT6835_CPR 2097152

#define MT6835_STATUS_OVERSPEED 0x01
#define MT6835_STATUS_WEAKFIELD 0x02
#define MT6835_STATUS_UNDERVOLT 0x04
#define MT6835_CRC_ERROR 0x08

#define MT6835_WRITE_ACK 0x55

#define MT6835_REG_USERID 0x001

#define MT6835_REG_ANGLE1 0x003
#define MT6835_REG_ANGLE2 0x004
#define MT6835_REG_ANGLE3 0x005
#define MT6835_REG_ANGLE4 0x006

#define MT6835_REG_ABZ_RES1 0x007
#define MT6835_REG_ABZ_RES2 0x008

#define MT6835_REG_ZERO1 0x009
#define MT6835_REG_ZERO2 0x00A

#define MT6835_REG_OPTS0 0x00A
#define MT6835_REG_OPTS1 0x00B
#define MT6835_REG_OPTS2 0x00C
#define MT6835_REG_OPTS3 0x00D
#define MT6835_REG_OPTS4 0x00E
#define MT6835_REG_OPTS5 0x011

// NLC table, 192 bytes
#define MT6835_REG_NLC_BASE 0x013
#define MT6835_REG_CAL_STATUS 0x113

#define MT6835_BITORDER MSBFIRST


// declare any 'global' variables here
DR16 dr16;
rm_CAN can;
static SPISettings settings(1000000, MT6835_BITORDER, SPI_MODE3);

Timer loop_timer;

PIDFilter calib_motor_pid;

// DONT put anything else in this function. It is not a setup function
void print_logo() {
    if (Serial) {
        Serial.println("TEENSY SERIAL START\n\n");
        Serial.print("\033[1;33m");
        Serial.println("                  .:^!?!^.                        ");
        Serial.println("           .:~!?JYYYJ?7?Y5Y7!!.                   ");
        Serial.println("         :?5YJ?!~:.      ^777YP?.                 ");
        Serial.println("         5G~                  ~YP?:               ");
        Serial.println("         7P5555Y:               ^YP?:....         ");
        Serial.println("        ~55J7~^.   ..    .        ^JYYYYYYYYYJJ!. ");
        Serial.println("        YG^     !Y5555J:^PJ    Y5:      ...::^5G^ ");
        Serial.println("       :GY    .YG?^..^~ ~GY    5G^ ^!~~^^^!!~7G?  ");
        Serial.println(" .!JYYY5G!    7BJ       ~GY    5G^ ~??JJJY555GP!  ");
        Serial.println("^55!^:.^~.    ^PP~   .: ^GP:  ^PP:           :7PY.");
        Serial.println("YG^            :JP5YY55: ~YP55PY^              ~GJ");
        Serial.println("?G~      .?7~:   .^~~^.    .^:.                :G5");
        Serial.println(".5P^     7BYJ5YJ7^.                          .~5P^");
        Serial.println(" .JPJ!~!JP?  .:~?PP^            .:.    .^!JYY5Y!. ");
        Serial.println("   :!???!:       5P.         .!Y5YYYJ?Y5Y?!^:.    ");
        Serial.println("                 7G7        7GY!. .:~!^.          ");
        Serial.println("                  JG!      :G5                    ");
        Serial.println("                   7PY!^^~?PY:                    ");
        Serial.println("                    .!JJJJ?^                      ");
        Serial.print("\033[0m");
        Serial.println("\n\033[1;92mFW Ver. 999.999.999");
        Serial.printf("\nLast Built: %s at %s", __DATE__, __TIME__);
        Serial.printf("\nRandom Num: %x", ARM_DWT_CYCCNT);
        Serial.println("\033[0m\n");
    }
}

// Master loop
int main() {
    Serial.begin(1000000); // the serial monitor is actually always active (for debug use Serial.println & tycmd)
    print_logo();

    // initialize any 'setup' functions here
    pinMode(13, OUTPUT);
    dr16.init();
    can.init();

    int nCS = 37; // 37 or 36 (enc 1, enc 2)
    int nCAL = 5; // 5 or 6 (enc 1, enc 2)
    pinMode(nCS, OUTPUT);
    pinMode(nCAL, OUTPUT);
    digitalWrite(nCS, HIGH);
    Serial.println("Starting SPI");
    SPI.begin();
    Serial.println("SPI Started");

    calib_motor_pid.K[0] = 0.0002;

    // main loop
    while (true) {
        dr16.read();
        can.read();

        if (!dr16.is_connected() || dr16.get_l_switch() == 1) {
            // SAFETY ON
            can.zero();
            can.zero_motors();
            digitalWrite(6, LOW);

            //// READ
            uint8_t data[6] = {0}; // transact 48 bits
            data[0] = (MT6835_OP_ANGLE<<4);
            data[1] = MT6835_REG_ANGLE1;
            SPI.beginTransaction(settings);
            digitalWrite(nCS, LOW);
            SPI.transfer(data, 6);
            digitalWrite(nCS, HIGH);
            SPI.endTransaction();
            int raw_angle = (data[2] << 13) | (data[3] << 5) | (data[4] >> 3);
            float radians = raw_angle / (float)MT6835_CPR * (3.14159265*2.0);
            float degrees = radians * (180/3.14159265);
            ////
            Serial.printf("%d raw      %0.8f degrees      %0.8f radians\n", raw_angle, degrees, radians);
        } else {//if (dr16.is_connected() && dr16.get_l_switch() == 3) {
            // SAFETY OFF
            digitalWrite(nCAL, HIGH);

            //// CALIBRATE
            uint8_t data[3] = {0};
            data[0] = MT6835_OP_READ << 4 | MT6835_REG_CAL_STATUS >> 8;
            data[1] = MT6835_REG_CAL_STATUS;

            SPI.beginTransaction(settings);
            digitalWrite(nCS, LOW);
            SPI.transfer(data, 3);
            digitalWrite(nCS, HIGH);
            SPI.endTransaction();

            int calib_mode = data[2] >> 6;

            ////

            float motor_speed = can.get_motor_attribute(CAN_2, 1, MotorAttribute::SPEED);
            calib_motor_pid.setpoint = 4800;
            calib_motor_pid.measurement = motor_speed;
            float output = calib_motor_pid.filter(0.001);
            can.write_motor_norm(CAN_2, 1, C620, output);
            Serial.printf("CALIBRATING! %f motor rpm      %d calibration mode\n", motor_speed, calib_mode);
            ////

            can.write();
        }

        // LED heartbeat
        millis() % 500 < 100 ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);

        // Keep the loop running at 1kHz
        loop_timer.delay_millis(1);
    }

    return 0;
}


// Calibrated at 4800 RPM