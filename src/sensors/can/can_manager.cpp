#include "can_manager.hpp"

// driver includes are here not in header since they're only needed in the implementation
// TODO: is this a good decision?
#include "can/C610.hpp"
#include "can/C620.hpp"
#include "can/MG8016EI6.hpp"

CANManager::CANManager() { }

CANManager::~CANManager() {
    // go through and delete any allocated motors
    for (uint32_t motor = 0; motor < CAN_MAX_MOTORS; motor++) {
        if (m_motors[motor] != nullptr) {
            delete m_motors[motor];
        }

        m_motors[motor] = nullptr;
    }
}

void CANManager::init() {
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

void CANManager::configure(float motor_info[CAN_MAX_MOTORS][3]) {
    // loop through all CAN_MAX_MOTORS
    for (uint32_t motor_id = 0; motor_id < CAN_MAX_MOTORS; motor_id++) {
        // grab the information for this specific motor
        int controller_type = (int)motor_info[motor_id][0];
        int physical_id = (int)motor_info[motor_id][1];
        int bus_id = (int)motor_info[motor_id][2];

        // if the controller type is 0, then this motor is unused and we should process the next one
        if (controller_type == 0) {
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
            Serial.printf("CANManager tried to create a motor of invalid type: %d\n", controller_type);
            break;
        }
        }

        // place the new motor in the array
        m_motors[motor_id] = new_motor;
    }
}

void CANManager::read() {
    // for each bus
    for (uint32_t bus = 0; bus < CAN_NUM_BUSSES; bus++) {
        // we want to read all the messages from this bus as there might be many queued up
        CAN_message_t msg;
        while (m_busses[bus]->read(msg)) {
            // distribute the message to the correct motor
            distribute_msg(msg);
        }
    }
}

void CANManager::write() {
    // for each bus
    for (uint32_t bus = 0; bus < CAN_NUM_BUSSES; bus++) {
        // the c610s and c620s require combined messages so are treated differently
        // I refer to them as rm motors as they are from RoboMaster
        // first msg is for the first 4 motors, second msg is for the last 4 motors
        // both the c610s and c620s can occupy the same message
        CAN_message_t rm_motor_msgs[2];

        // for each motor
        for (uint32_t motor = 0; motor < CAN_MAX_MOTORS; motor++) {
            // if the motor is null, skip it
            if (m_motors[motor] == nullptr) {
                continue;
            }

            // if the motor is on the wrong bus, skip it
            if (m_motors[motor]->get_bus_id() != bus) {
                continue;
            }

            // based on the motor type, figure out how to write the message
            switch (m_motors[motor]->get_controller_type()) {
            case C610_CONTROLLER:   // fallthrough
            case C620_CONTROLLER: {
                // depending on the motor ID, write the message to the correct msg in the array
                if ((m_motors[motor]->get_id() - 1) / 4) {
                    m_motors[motor]->write(rm_motor_msgs[1]);   // last 4 motors
                } else {
                    m_motors[motor]->write(rm_motor_msgs[0]);   // first 4 motors
                }

                // this combined message will be written to the bus after the motor loop

                break;
            }
            case MG8016_CONTROLLER: {
                CAN_message_t msg;

                // get its message data
                m_motors[motor]->write(msg);
                
                // the MG motors dont require msg merging so just write it to the bus
                // write the message to the correct bus
                m_busses[bus]->write(msg);

                break;
            }
            default: {
                Serial.printf("CANManager tried to write to a motor of invalid type: %d\n", m_motors[motor]->get_controller_type());
                break;
            }
            }
        }
        
        // write the combined messages to the bus
        m_busses[bus]->write(rm_motor_msgs[0]);
        m_busses[bus]->write(rm_motor_msgs[1]);
    }

}

void CANManager::safety_mode() {
    // for each motor
    for (uint32_t motor = 0; motor < CAN_MAX_MOTORS; motor++) {
        // if the motor is null, skip it
        if (m_motors[motor] == nullptr) {
            continue;
        }

        // call the motor's zero function
        m_motors[motor]->zero_motor();
    }

    // write the zero torque commands to the bus
    write();
}

void CANManager::write_motor_torque(uint32_t motor_gid, float torque) {
    // verify motor ID
    if (motor_gid >= CAN_MAX_MOTORS) {
        Serial.printf("CANManager tried to write to an invalid motor: %d\n", motor_gid);
        return;
    }

    // verify motor exists
    if (m_motors[motor_gid] == nullptr) {
        Serial.printf("CANManager tried to write to an invalid motor: %d\n", motor_gid);
        return;
    }

    // write the torque to the motor
    m_motors[motor_gid]->write_motor_torque(torque);
}

void CANManager::print_state() {
    // for each motor
    for (uint32_t motor = 0; motor < CAN_MAX_MOTORS; motor++) {
        // if the motor is null, skip it
        if (m_motors[motor] == nullptr) {
            continue;
        }

        // print the motor state
        m_motors[motor]->print_state();

        Serial.println();
    }
}

void CANManager::print_motor_state(uint32_t motor_gid) {
    // verify motor ID
    if (motor_gid > CAN_MAX_MOTORS) {
        Serial.printf("CANManager tried to print an invalid motor: %d\n", motor_gid);
        return;
    }

    // verify motor exists
    if (m_motors[motor_gid] == nullptr) {
        Serial.printf("CANManager tried to print an invalid motor: %d\n", motor_gid);
        return;
    }

    // print the motor state
    m_motors[motor_gid]->print_state();
}

Motor* CANManager::get_motor(uint32_t motor_gid) {
    // verify motor ID is not out of bounds
    if (motor_gid > CAN_MAX_MOTORS) {
        Serial.printf("CANManager tried to get an invalid motor: %d\n", motor_gid);
        return nullptr;
    }
    
    return m_motors[motor_gid];
}

bool CANManager::distribute_msg(CAN_message_t& msg) {
    // for each motor
    for (uint32_t motor = 0; motor < CAN_MAX_MOTORS; motor++) {
        // if the motor is null, skip it
        if (m_motors[motor] == nullptr) {
            continue;
        }

        // if the motor is on the wrong bus, skip it
        // msg.bus is 1-indexed but the motor bus_id is 0-indexed
        if (m_motors[motor]->get_bus_id() != (msg.bus - 1)) {
            continue;
        }

        // if the motor can handle the message, give it to the motor
        if (m_motors[motor]->read(msg)) {
            return true;
        }
    }
    
    return false;
}
