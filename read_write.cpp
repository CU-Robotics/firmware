#include "read_write.hpp"

void ReadWrite::ReadWrite() {
    
}

void ReadWrite::init() {
    can1.begin();
    can1.setBaudRate(1000000);
    can1.enableFIFO();

    can2.begin();
    can2.setBaudRate(1000000);
    can2.enableFIFO();
}

int ReadWrite::getAngle(int motID, int canNum) {
    return (input[canNum][motID][0] << 8) | input[canNum][motID][1];
}

int ReadWrite::getRotSpd(int motID, int canNum) {
    return (input[canNum][motID][2] << 8) | input[canNum][motID][3];
}

int ReadWrite::getTrqCur(int motID, int canNum) {
    return (input[canNum][motID][4] << 8) | input[canNum][motID][5];
}

void ReadWrite::readCAN() {
    CAN_message_t msg;

    while (can1.read(msg)) {

        int index = msg.id - 0x201;
        for (int i = 0; i < 8; i++)
            input[0][index][i] = msg.buf[i];
    }

    while (can2.read(msg)) {

        int index = msg.id - 0x200;
        for (int i = 0; i < 8; i++)
            input[1][index][i] = msg.buf[i];
    }
}

void ReadWrite::writeCAN() {
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

void ReadWrite::zeroCAN() {
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

void ReadWrite::writeMtr(short val, int motID, int canNum) {
    union sto2c {
        short num = val;
        struct {
            char c1;
            char c2;
        }
    } sto2c conv;
    output[canNum][motID*2-1] = conv.c1;
    output[canNum][motID*2] = conv.c2;
}