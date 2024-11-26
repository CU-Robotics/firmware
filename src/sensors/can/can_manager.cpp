#include "can_manager.hpp"

// driver includes are here not in header since they're only needed in the implementation
// TODO: is this a good decision?
#include "can/C610.hpp"
#include "can/C620.hpp"
#include "can/MG8016EI6.hpp"

void CanManager::init() {
    // initialize CAN 1
    m_can1.begin();
    m_can1.setBaudRate(1000000u);   // 1Mbit baud
    // TODO: fifo?

    // initialize CAN 2
    m_can2.begin();
    m_can2.setBaudRate(1000000u);   // 1Mbit baud

    // initialize CAN 3
    // TODO: can CAN 3 act the same as CAN 1/2 since its CANFD?
    m_can3.begin();
    m_can3.setBaudRate(1000000u);   // 1Mbit baud

    // destroy any motors in existance and initialize to nullptr
    for (uint32_t i = 0; i < CAN_MAX_MOTORS; i++) {
        if (m_motors[i] != nullptr)
            delete m_motors[i];
        
        m_motors[i] = nullptr;
    }
}

void CanManager::configure(float motor_info[CAN_MAX_MOTORS][3]) {
    // loop through all CAN_MAX_MOTORS
    for (uint32_t motor_id = 0; motor_id < CAN_MAX_MOTORS; motor_id++) {
        // grab the information for this specific motor
        int controller_type = (int)motor_info[motor_id][0];
        int physical_id = (int)motor_info[motor_id][1];
        int bus_id = (int)motor_info[motor_id][2];

        // if the controller type is 0, then this motor is unused and we should process the next one
        if (controller_type == -1) {
            continue;
        }
        // else this motor is valid

        // create the motor based on the controller type
        Motor* new_motor = nullptr;

        switch (controller_type) {
        case C610_CONTROLLER: {
            Serial.printf("Creating C610 Motor: %d\n", motor_id);
            new_motor = new C610(motor_id, physical_id, bus_id);
            break;
        }
        case C620_CONTROLLER: {
            Serial.printf("Creating C620 Motor: %d\n", motor_id);
            new_motor = new C620(motor_id, physical_id, bus_id);
            break;
        }
        case MG8016_CONTROLLER: {
            Serial.printf("Creating MG Motor: %d\n", motor_id);
            new_motor = new MG8016EI6(motor_id, physical_id, bus_id);
            break;
        }
        default: {
            Serial.printf("CanManager tried to create a motor of invalid type: %d\n", controller_type);
            break;
        }
        }
    }
}