#include "can_manager.hpp"

// driver includes are here not in header since they're only needed in the implementation
#include "can/C610.hpp"
#include "can/C620.hpp"
#include "can/MG8016EI6.hpp"
#include "can/GIM.hpp"
#include "can/SDC104.hpp"

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
    m_can1.enableFIFO(true);
    // TODO: fifo?

    // initialize CAN 2
    m_can2.begin();
    m_can2.setBaudRate(1000000u);   // 1Mbit baud
    m_can2.enableFIFO(true);

    // initialize CAN 3
    // TODO: can CAN 3 act the same as CAN 1/2 since its CANFD?
    m_can3.begin();
    m_can3.setBaudRate(1000000u);   // 1Mbit baud
    m_can3.enableFIFO(true);

    // destroy any motors in existance and initialize to nullptr
    m_motor_map.clear();
}

void CANManager::configure(const float motor_info[CAN_MAX_MOTORS][4]) {
    // using the motor_info array, create the motors following the config
    
    // loop through all CAN_MAX_MOTORS
    for (uint32_t motor_id = 0; motor_id < CAN_MAX_MOTORS; motor_id++) {
        // grab the information for this specific motor
        int controller_type = (int)motor_info[motor_id][0];
        int physical_id = (int)motor_info[motor_id][1];
        int bus_id = (int)motor_info[motor_id][2];
        MotorType motor_type = (MotorType)motor_info[motor_id][3];

        // if the controller type is 0, then this motor is unused and we should process the next one
        if (static_cast<MotorControllerType>(controller_type) == MotorControllerType::NULL_MOTOR_CONTROLLER_TYPE) {
            continue;
        }
        // else this motor is valid

        // create the motor based on the controller type
        Motor* new_motor = nullptr;

        switch (static_cast<MotorControllerType>(controller_type)) {
        case MotorControllerType::C610: {
            new_motor = new C610(motor_id, physical_id, bus_id, motor_type);
            Serial.printf("Creating C610 Motor: %d on bus %d\n", new_motor->get_id(), new_motor->get_bus_id(), new_motor->get_motor_type());
            break;
        }
        case MotorControllerType::C620: {
            new_motor = new C620(motor_id, physical_id, bus_id, motor_type);
            Serial.printf("Creating C620 Motor: %d on bus %d\n", new_motor->get_id(), new_motor->get_bus_id(), new_motor->get_motor_type());
            break;
        }
        case MotorControllerType::MG8016: {
            new_motor = new MG8016EI6(motor_id, physical_id, bus_id, motor_type);
            Serial.printf("Creating MG Motor: %d on bus %d\n", new_motor->get_id(), new_motor->get_bus_id(), new_motor->get_motor_type());
            break;
        }
        case MotorControllerType::GIM: {
            new_motor = new GIM(motor_id, physical_id, bus_id, motor_type);
            Serial.printf("Creating GIM Motor: %d on bus %d\n", new_motor->get_id(), new_motor->get_bus_id(), new_motor->get_motor_type());
            break;
        }
        case MotorControllerType::SDC104: {
            new_motor = new SDC104(motor_id, physical_id, bus_id, motor_type);
            Serial.printf("Creating SDC104 Motor: %d on bus %d\n", new_motor->get_id(), new_motor->get_bus_id(), new_motor->get_motor_type());
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
                // - 1 on msg.bus to maintain bus IDs being 0-indexed
                Serial.printf("CANManager failed to distribute message with raw CAN ID: %.4x on bus: %x\n", msg.id, msg.bus - 1);
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

        // flag to see whether rm motors are even on this bus
        // we dont want to send a write command to motors that wont be on this bus
        bool should_send_rm_motors = false;

        // for each motor, can be const
        for (const auto& motor : m_motor_map) {
            // if the motor is not on this bus, skip it
            if (motor.second->get_bus_id() != bus) {
                continue;
            }
            
            // based on the motor type, figure out how to write the message
            switch (motor.second->get_controller_type()) {
            case MotorControllerType::C610:   // fallthrough
            case MotorControllerType::C620: {
                // depending on the motor ID, write the message to the correct msg in the array
                // - 1 to get the id into 0-indexed form, then divide by 4 to get the upper or lower half as an index (0, 1)
                if ((motor.second->get_id() - 1) / 4) {
                    motor.second->write(rm_motor_msgs[1]);   // last 4 motors
                } else {
                    motor.second->write(rm_motor_msgs[0]);   // first 4 motors
                }

                // this combined message will be written to the bus after the motor loop

                // there are rm motors on this bus that need to be written
                should_send_rm_motors = true;

                break;
            }
            case MotorControllerType::MG8016:
            case MotorControllerType::GIM:
            case MotorControllerType::SDC104: {
                // these motors dont require msg merging so just write it to the bus
                CAN_message_t msg;

                // get its message data
                motor.second->write(msg);
                
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
        
        // write the rm motor messages to the bus
        if (should_send_rm_motors) {
            m_busses[bus]->write(rm_motor_msgs[0]);
            m_busses[bus]->write(rm_motor_msgs[1]);
        }
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
    #ifdef CAN_MANAGER_DEBUG
        Serial.printf("CANManager tried to write to an invalid motor: %d on bus %d\n", motor_id, bus_id);
    #endif
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
    #ifdef CAN_MANAGER_DEBUG
        Serial.printf("CANManager tried to print an invalid motor: %d on bus %d\n", motor_id, bus_id);
    #endif
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
#ifdef CAN_MANAGER_DEBUG
    Serial.printf("CANManager tried to get an invalid motor: %d on bus %d\n", motor_id, bus_id);
#endif

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
    #ifdef CAN_MANAGER_DEBUG
        Serial.printf("CANManager tried to get the state of an invalid motor: %d on bus %d\n", motor_id, bus_id);
    #endif
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

    // maintain a list of motors that have been initialized, indexed by motor global id
    bool motors_initialized[CAN_MAX_MOTORS] = { false };

    // only run the initialization for the timeout time
    uint32_t start_time = millis();

    while (millis() - start_time < m_motor_init_timeout) {
        // for each bus
        for (uint32_t bus = 0; bus < CAN_NUM_BUSSES; bus++) {
            CAN_message_t msg;

            // we want to read all the messages from this bus as there might be many queued up
            while (m_busses[bus]->read(msg)) {
                // try to distribute the message to the correct motor
                Motor* motor = distribute_msg(msg);

                // no motor could handle the message so move on
                if (!motor)
                    continue;

                // if the motor is not initialized, mark it as initialized
                if (!motors_initialized[motor->get_global_id()]) {
                    motors_initialized[motor->get_global_id()] = true;
                #ifdef CAN_MANAGER_DEBUG
                    Serial.printf("Motor %u on bus %u initialized!\n", motor->get_id(), motor->get_bus_id());
                #endif
                }
            }
        }
    }

    // print out any motors that failed to initialize using the motors_initialized array
    // this is not fatal but should be investigated
    for (auto& motor : m_motor_map) {
        if (!motors_initialized[motor.second->get_global_id()]) {
            Serial.printf("Motor %u on bus %u failed to initialize!\n", motor.second->get_id(), motor.second->get_bus_id());
        }
    }
}

Motor* CANManager::distribute_msg(CAN_message_t& msg) {
    // for each motor, cant be const
    for (auto& motor : m_motor_map) {
        // if the motor can handle the message, give it to the motor
        if (motor.second->read(msg)) {
            return motor.second;
        }
    }
    
    // no motors could handle the message
    return nullptr;
}
