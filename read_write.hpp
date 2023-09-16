#include <FlexCAN_T4.h>

#ifndef __ReadWrite__
#define __ReadWrite__

class ReadWrite {
private:
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

    int output[2][16];
    int input[2][8][8];


public:
    void ReadWrite();
    

    void init();

    int getAngle(int motID, int canNum);
    int getRotSpd(int motID, int canNum);
    int getTrqCur(int motID, int canNum);

    void readCAN();
    void writeCAN();
    void printCAN(int mtrID, int canID);

    void zeroCAN();
    void writeMtr(short val, int motID, int canNum);

};

#endif