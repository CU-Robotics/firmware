#include "rm_can.hpp"

rm_CAN::rm_CAN() {}

void rm_CAN::init() {
    // initialize CAN 1
    m_can1.begin();
    m_can1.setBaudRate(1000000);
    m_can1.enableFIFO(true);

    // initialize CAN 2
    m_can2.begin();
    m_can2.setBaudRate(1000000);
    m_can2.enableFIFO(true);

    // set message IDs for CAN 1
    m_output[CAN_1][0].id = 0x200;
    m_output[CAN_1][1].id = 0x1ff;
    m_output[CAN_1][2].id = 0x2ff;

    // set message IDs for CAN 2
    m_output[CAN_2][0].id = 0x200;
    m_output[CAN_2][1].id = 0x1ff;
    m_output[CAN_2][2].id = 0x2ff;

    // zero CANs just in case
    zero();
}

void rm_CAN::read() {
    // read from CAN 1
    CAN_message_t msg1;

    while (m_can1.read(msg1)) {
        // isolate the ID part of the message id (0x202 becomes 2)
        int id = msg1.id & 0xf;
        id -= 1; // subtract by 1 to allow for array indexing
        
        // fill appropriate buffer
        for (int i = 0; i < CAN_MESSAGE_SIZE; i++) m_input[CAN_1][id][i] = msg1.buf[i];
    }

    // read from CAN 2
    CAN_message_t msg2;
    
    while (m_can2.read(msg2)) {
        // isolate the ID part of the message id (0x202 becomes 2)
        int id = msg2.id & 0xf;
        id -= 1; // subtract by 1 to allow for array indexing
        
        // fill appropriate buffer
        for (int i = 0; i < CAN_MESSAGE_SIZE; i++) m_input[CAN_2][id][i] = msg2.buf[i];
    }
}

uint8_t rm_CAN::write() {
    int w1, w2;
    
    // write all messages to CAN 1
    for (int i = 0; i < NUM_MESSAGE_IDS; i++)
        w1 = m_can1.write(m_output[CAN_1][i]);

    // write all messages to CAN 2
    for (int i = 0; i < NUM_MESSAGE_IDS; i++)
        w2 = m_can2.write(m_output[CAN_2][i]);

    // normalize from the -1 or 1 output of CAN write command to 0 or 1
    if (w1 == -1)
        w1 = 0;
    if (w2 == -1)
        w2 = 0;

    return w1 && w2;
}

void rm_CAN::zero_motors() {
    // go through the entire m_output array and zero the buffers
    for (int i = 0; i < NUM_CAN_BUSES; i++) {
        for (int j = 0; j < NUM_MESSAGE_IDS; j++) {
            for (int k = 0; k < CAN_MESSAGE_SIZE; k++) {
                m_output[i][j].buf[k] = 0;
            }
        }
    }
}

void rm_CAN::zero() {
    // zero all output messages
    zero_motors();

    // write all output messages (now all 0s)
    write();
}

void rm_CAN::write_motor(uint16_t canID, uint16_t motorID, int32_t value) {
    // find the corresponding message ID based on the motor ID
    // this division will truncate motors: 
    //      [0 - 3]  to index 0 (ID: 0x200)
    //      [4 - 7]  to index 1 (ID: 0x1ff)
    //      [8 - 11] to index 2 (ID: 0x2ff)
    uint8_t messageID = (motorID-1) / 4;

    // mID just finds the 'actual' ID based on what index it should be
    uint8_t mID = (motorID-1) % 4;

    // set to buffer indexes (mID * 2) and (miD * 2 + 1) corresponding to the
    // expected output by the motors. Explained in motor documentation (page 14-16)

    // set big byte to correct index in buffer
    m_output[canID][messageID].buf[mID * 2] = (value >> 8) & 0xff;
    // set small byte to correct index in buffer
    m_output[canID][messageID].buf[mID * 2 + 1] = value & 0xff;
}

void rm_CAN::write_motor_norm(uint16_t canID, uint16_t motorID, uint8_t controllerType, float value) {
    switch (controllerType) {
        case C610:
            write_motor(canID, motorID, (int)(value*C610_OUTPUT_SCALE));
            break;
        case C620:
            write_motor(canID, motorID, (int)(value*C620_OUTPUT_SCALE));
            break;
        case GM6020:
            write_motor(canID, motorID, (int)(value*GM6020_OUTPUT_SCALE));
            break;
        default:
            Serial.println("CURo WARN: Invalid motor controller type on write_motor_norm() call!");
    }
}

int rm_CAN::get_motor_attribute(uint16_t canID, uint16_t motorID, MotorAttribute valueType) {
    // return correct value depending on valueType enum
    switch (valueType) {
        case MotorAttribute::ANGLE:
            return (uint16_t)(combine_bytes(m_input[canID][motorID-1][0], m_input[canID][motorID-1][1]));
            break;
        case MotorAttribute::SPEED:
            return (int16_t)(combine_bytes(m_input[canID][motorID-1][2], m_input[canID][motorID-1][3]));
            break;
        case MotorAttribute::TORQUE:
            return (int16_t)(combine_bytes(m_input[canID][motorID-1][4], m_input[canID][motorID-1][5]));
            break;
        case MotorAttribute::TEMP:
            return (uint16_t)(m_input[canID][motorID-1][6]);
            break;

        default:
            return 0;
            break;
    }
}

void rm_CAN::print_motor(uint16_t canID, uint16_t motorID, bool showFullHex) {
    // strip angle, speed, torque, and temp from input array
    uint16_t angle = combine_bytes(m_input[canID][motorID-1][0], m_input[canID][motorID-1][0 + 1]);
    int16_t speed = combine_bytes(m_input[canID][motorID-1][2], m_input[canID][motorID-1][2 + 1]);
    int16_t torque = combine_bytes(m_input[canID][motorID-1][4], m_input[canID][motorID-1][4 + 1]);
    uint16_t temp = m_input[canID][motorID-1][6];

    // speed and torque are interpreted as signed values. all others are unsigned

    if (showFullHex)
        Serial.printf("CAN: %x\tMotor: %x\t\t%.4x\t%.4x | %.4x\t%.4x | %.4x\t%.4x | %.4x\t%.4x\n", canID + 1, motorID-1 + 1, (angle >> 8) & 0xff, angle & 0xff, (speed >> 8) & 0xff, speed & 0xff, (torque >> 8) & 0xff, torque & 0xff, temp, 0);
    else 
        Serial.printf("CAN: %x\tMotor: %x\t\tAngle: %.4u\tSpeed: %.5d\tTorque: %.5d\tTemp: %.2u\n", canID + 1, motorID-1 + 1, angle, speed, torque, temp);
}

void rm_CAN::print_can(uint16_t canID, bool showFullHex) {
    // for all motors, print that motor
    for (int i = 0; i < NUM_MOTORS_PER_BUS; i++)
        print_motor(canID, i, showFullHex);
}

void rm_CAN::print_output() {
  for (int i = 0; i < NUM_CAN_BUSES; i++) {
    Serial.printf("CAN: %x\n", i);
    
    for (int j = 0; j < NUM_MESSAGE_IDS; j++) {
      Serial.printf("Message: %x\t", m_output[i][j].id);
      
      for (int k = 0; k < CAN_MESSAGE_SIZE; k++) {
        Serial.printf("%x ", m_output[i][j].buf[k]);
      }
      Serial.println();
    }
  }
  Serial.println();
}

uint16_t combine_bytes(uint8_t high, uint8_t low) {
  uint16_t result = 0;
  return ((result | high) << 8) | low;
}