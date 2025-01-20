#include "can_manager.hpp"

// driver includes are here not in header since they're only needed in the implementation
#include "can/C610.hpp"
#include "can/C620.hpp"
#include "can/MG8016EI6.hpp"

// FlexCAN_T4 moment
CANManager::CANManager() { }

CANManager::~CANManager() {
    // go through and delete any allocated motors
    m_motor_map.clear();
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
    m_motor_map.clear();
}

void CANManager::configure(float motor_info[CAN_MAX_MOTORS][3]) {
    // using the motor_info array, create the motors following the config
    
    // loop through all CAN_MAX_MOTORS
    for (uint32_t motor_id = 0; motor_id < CAN_MAX_MOTORS; motor_id++) {
        // grab the information for this specific motor
        int controller_type = (int)motor_info[motor_id][0];
        int physical_id = (int)motor_info[motor_id][1];
        int bus_id = (int)motor_info[motor_id][2];

        // if the controller type is 0, then this motor is unused and we should process the next one
        if (static_cast<MotorControllerType>(controller_type) == MotorControllerType::NULL_MOTOR_CONTROLLER_TYPE) {
            continue;
        }
        // else this motor is valid

        // create the motor based on the controller type
        Motor* new_motor = nullptr;

        switch (static_cast<MotorControllerType>(controller_type)) {
        case MotorControllerType::C610_CONTROLLER: {
            Serial.printf("Creating C610 Motor: %d on bus %d\n", motor_id, bus_id);
            new_motor = new C610(motor_id, physical_id, bus_id);
            break;
        }
        case MotorControllerType::C620_CONTROLLER: {
            Serial.printf("Creating C620 Motor: %d on bus %d\n", motor_id, bus_id);
            new_motor = new C620(motor_id, physical_id, bus_id);
            break;
        }
        case MotorControllerType::MG8016_CONTROLLER: {
            Serial.printf("Creating MG Motor: %d on bus %d\n", motor_id, bus_id);
            new_motor = new MG8016EI6(motor_id, physical_id, bus_id);
            break;
        }
        default: {
            Serial.printf("CANManager tried to create a motor of invalid type: %d\n", controller_type);
            continue;   // continue in order to not call the later map insert since new_motor would be null
        }
        }

        // place the new motor in the map
        m_motor_map.insert({ motor_id, new_motor });
    }

    // verify all motors are online and ready
    init_motors();
}

void CANManager::read() {
    // for each bus
    for (uint32_t bus = 0; bus < CAN_NUM_BUSSES; bus++) {
        // we want to read all the messages from this bus as there might be many queued up
        CAN_message_t msg;
        while (m_busses[bus]->read(msg)) {
            // distribute the message to the correct motor
            // if this fails, we've received a message that does not match any motor
            // how would this happen?
            if (!distribute_msg(msg)) {
                Serial.printf("CANManager failed to distribute message: %d\n", msg.id);
            }
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

        // for each motor, can be const
        for (const auto& motor : m_motor_map) {
            // based on the motor type, figure out how to write the message
            switch (motor.second->get_controller_type()) {
            case MotorControllerType::C610_CONTROLLER:   // fallthrough
            case MotorControllerType::C620_CONTROLLER: {
                // depending on the motor ID, write the message to the correct msg in the array
                // - 1 to get the id into 0-indexed form, then divide by 4 to get the upper or lower half as an index (0, 1)
                if ((motor.second->get_id() - 1) / 4) {
                    motor.second->write(rm_motor_msgs[1]);   // last 4 motors
                } else {
                    motor.second->write(rm_motor_msgs[0]);   // first 4 motors
                }

                // this combined message will be written to the bus after the motor loop

                break;
            }
            case MotorControllerType::MG8016_CONTROLLER: {
                CAN_message_t msg;

                // get its message data
                motor.second->write(msg);
                
                // the MG motors dont require msg merging so just write it to the bus
                // write the message to the correct bus
                m_busses[bus]->write(msg);

                break;
            }
            default: {
                Serial.printf("CANManager tried to write to a motor of invalid type: %d\n", motor.second->get_controller_type());
                break;
            }
            }
        }
        
        // write the combined messages to the bus
        m_busses[bus]->write(rm_motor_msgs[0]);
        m_busses[bus]->write(rm_motor_msgs[1]);
    }
}

void CANManager::issue_safety_mode() {
    // for each motor, cant be const
    for (auto& motor : m_motor_map) {
        motor.second->zero_motor();
    }

    // write the zero torque commands to the bus
    write();
}

void CANManager::write_motor_torque(uint32_t motor_gid, float torque) {
    // verify motor ID
    if (m_motor_map.count(motor_gid) == 0) {
        Serial.printf("CANManager tried to write to an invalid motor: %d\n", motor_gid);
        return;
    }

    // write the torque to the motor
    m_motor_map[motor_gid]->write_motor_torque(torque);
}

void CANManager::write_motor_torque(uint32_t bus_id, uint32_t motor_id, float torque) {
    // find the motor by bus and ID
    Motor* motor = get_motor(bus_id, motor_id);
    if (!motor) {
        Serial.printf("CANManager tried to write to an invalid motor: %d on bus %d\n", motor_id, bus_id);
        return;
    }

    // write the torque to the motor
    motor->write_motor_torque(torque);
}

void CANManager::print_state() {
    // for each motor, print it's state
    for (const auto& motor : m_motor_map) {
        // print the motor state
        motor.second->print_state();

        Serial.println();
    }
}

void CANManager::print_motor_state(uint32_t motor_gid) {
    // verify motor ID
    if (m_motor_map.count(motor_gid) == 0) {
        Serial.printf("CANManager tried to print an invalid motor: %d\n", motor_gid);
        return;
    }

    // print the motor state
    m_motor_map[motor_gid]->print_state();
}

void CANManager::print_motor_state(uint32_t bus_id, uint32_t motor_id) {
    // find the motor by bus and ID
    Motor* motor = get_motor(bus_id, motor_id);
    if (!motor) {
        Serial.printf("CANManager tried to print an invalid motor: %d on bus %d\n", motor_id, bus_id);
        return;
    }

    // print the motor state
    motor->print_state();
}

Motor* CANManager::get_motor(uint32_t motor_gid) {
    // verify motor ID
    if (m_motor_map.count(motor_gid) == 0) {
        Serial.printf("CANManager tried to get an invalid motor: %d\n", motor_gid);
        return nullptr;
    }

    // return the motor
    return m_motor_map[motor_gid];
}

Motor* CANManager::get_motor(uint32_t bus_id, uint32_t motor_id) {
    // for each motor, can be const
    for (const auto& motor : m_motor_map) {
        if (motor.second->get_bus_id() == bus_id && motor.second->get_id() == motor_id) {
            return motor.second;
        }
    }

    // could not find the motor
    Serial.printf("CANManager tried to get an invalid motor: %d on bus %d\n", motor_id, bus_id);

    return nullptr;
}

MotorState CANManager::get_motor_state(uint32_t motor_gid) {
    // verify motor ID
    if (m_motor_map.count(motor_gid) == 0) {
        Serial.printf("CANManager tried to get the state of an invalid motor: %d\n", motor_gid);
        return MotorState();
    }

    // return the motor state
    return m_motor_map[motor_gid]->get_state();
}

MotorState CANManager::get_motor_state(uint32_t bus_id, uint32_t motor_id) {
    // find the motor by bus and ID
    Motor* motor = get_motor(bus_id, motor_id);
    if (!motor) {
        Serial.printf("CANManager tried to get the state of an invalid motor: %d on bus %d\n", motor_id, bus_id);
        return MotorState();
    }

    // return the motor state
    return motor->get_state();
}

void CANManager::init_motors() {
    // all motors have been created, go through and verify we are getting data from them and call their init functions

    // for each motor, call it's init function, cant be const
    for (auto& motor : m_motor_map) {
        // call the motor's init function
        motor.second->init();
    }

    // issue a write command to the bus to push the init commands
    // this cant easily be placed in the same loop as the init functions since some motors dont allow single writes
    write();

    // wait for the motors to initialize or timeout

    // maintain a list of motors that have been initialized
    bool motors_initialized[CAN_MAX_MOTORS] = { false };

    // for each bus, read all messages until all motors have been initialized
    for (uint32_t bus = 0; bus < CAN_NUM_BUSSES; bus++) {
        CAN_message_t msg;

        // start the timeout timer
        uint32_t start_time = millis();
        
        // read all messages from the bus. exit if the timeout is reached and the buffer is empty
        while (millis() - start_time < m_motor_init_timeout || m_busses[bus]->read(msg)) {
            // for each motor, find the motor that this msg belongs to
            for (auto& motor : m_motor_map) {
                // mark the motor as initialized if the message was read
                if (motor.second->read(msg)) {
                    motors_initialized[motor.second->get_global_id()] = true;
                    Serial.printf("Motor %d initialized\n", motor);
                }
            }
        }
    }

    // print out any motors that failed to initialize using the motors_initialized array
    // this is not fatal but should be investigated
    for (uint32_t motor = 0; motor < CAN_MAX_MOTORS; motor++) {
        // if this motor is not initialized and it actually exists, print a warning
        if (!motors_initialized[motor] && m_motor_map.count(motor) != 0) {
            Serial.printf("Motor %d on bus %d with id %d failed to initialize within %ums\n", motor, m_motor_map[motor]->get_bus_id(), m_motor_map[motor]->get_id(), m_motor_init_timeout);
        }
    }
}

bool CANManager::distribute_msg(CAN_message_t& msg) {
    // for each motor, cant be const
    for (auto& motor : m_motor_map) {
        // if the motor can handle the message, give it to the motor
        if (motor.second->read(msg)) {
            return true;
        }
    }
    
    return false;
}
