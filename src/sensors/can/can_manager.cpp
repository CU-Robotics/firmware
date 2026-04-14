#include "can_manager.hpp"
#include "utils/logger.hpp"

// driver includes are here not in header since they're only needed in the implementation
#include "safety.hpp"
#include "sensors/can/C610.hpp"
#include "sensors/can/C620.hpp"
#include "sensors/can/MG8016EI6.hpp"
#include "sensors/can/GIM.hpp"
#include "sensors/can/SDC104.hpp"
#include "comms/data/motor_state_data.hpp"
#include "comms/data/sendable.hpp"
#include <cstdint>
#include <set>

#include "utils/safety.hpp"

// FlexCAN_T4 moment
CANManager::CANManager() { }

CANManager::~CANManager() {
    //clean up motors since they are allocated.
    m_motor_name_map.clear();
}

void CANManager::init(const std::vector<Cfg::Motor>& motor_configs) {
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
    m_motor_name_map.clear();
    
    for(const Cfg::Motor& motor_config : motor_configs) {
        configure_motor(motor_config);
    }

    init_motors();
}

void CANManager::configure_motor(const Cfg::Motor& motor_config){

    switch(motor_config.motor_controller_type) {
        case Cfg::MotorControllerType::C610: {
            m_motor_name_map.insert({ motor_config.motor_name, std::make_shared<C610>(motor_config) });
            Serial.printf("Creating C610 motor %u on bus %u with ID %u\n", static_cast<uint32_t>(motor_config.motor_name), motor_config.physical_bus, motor_config.physical_id);
            
            break;
        }
        case Cfg::MotorControllerType::C620: {
            m_motor_name_map.insert({ motor_config.motor_name, std::make_shared<C620>(motor_config) });
            Serial.printf("Creating C620 Motor %u on bus %u with ID %u\n", static_cast<uint32_t>(motor_config.motor_name), motor_config.physical_bus, motor_config.physical_id);
            break;
        }
        case Cfg::MotorControllerType::MG: {
            m_motor_name_map.insert({ motor_config.motor_name, std::make_shared<MG8016EI6>(motor_config) });
            Serial.printf("Creating MG Motor %u on bus %u with ID %u\n", static_cast<uint32_t>(motor_config.motor_name), motor_config.physical_bus, motor_config.physical_id);
            break;
        }
        case Cfg::MotorControllerType::GIM: {
            m_motor_name_map.insert({ motor_config.motor_name, std::make_shared<GIM>(motor_config) });
            Serial.printf("Creating GIM Motor %u on bus %u with ID %u\n", static_cast<uint32_t>(motor_config.motor_name), motor_config.physical_bus, motor_config.physical_id);
            break;
        }
        case Cfg::MotorControllerType::SDC104: {
            m_motor_name_map.insert({ motor_config.motor_name, std::make_shared<SDC104>(motor_config) });
            Serial.printf("Creating SDC104 Motor %u on bus %u with ID %u\n", static_cast<uint32_t>(motor_config.motor_name), motor_config.physical_bus, motor_config.physical_id);
            break;
        }
        default: {
            Serial.printf("CANManager tried to create a motor of invalid type: %u\n", motor_config.motor_controller_type);
            break;   // continue in order to not call the later map insert since new_motor would be null
        }
    }
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
            if (distribute_msg(msg) == Cfg::MotorName::UnsetMotorName) {
                // - 1 on msg.bus to maintain bus IDs being 0-indexed
                logger.printf(LogDestination::Serial, "CANManager failed to distribute message with raw CAN ID: %.4x on bus: %x\n", msg.id, msg.bus - 1);
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
        for (const auto& [name, motor] : m_motor_name_map) {
            // if the motor is not on this bus, skip it
            if (motor->get_bus_id() != bus) {
                continue;
            }
            
            // based on the motor type, figure out how to write the message
            switch (motor->get_controller_type()) {
            case Cfg::MotorControllerType::C610:   // fallthrough
            case Cfg::MotorControllerType::C620: {
                // depending on the motor ID, write the message to the correct msg in the array
                // - 1 to get the id into 0-indexed form, then divide by 4 to get the upper or lower half as an index (0, 1)
                if ((motor->get_id() - 1) / 4) {
                    motor->write(rm_motor_msgs[1]);   // last 4 motors
                } else {
                    motor->write(rm_motor_msgs[0]);   // first 4 motors
                }

                // this combined message will be written to the bus after the motor loop

                // there are rm motors on this bus that need to be written
                should_send_rm_motors = true;

                break;
            }
            case Cfg::MotorControllerType::MG:
            case Cfg::MotorControllerType::GIM:
            case Cfg::MotorControllerType::SDC104: {
                // these motors dont require msg merging so just write it to the bus
                CAN_message_t msg;

                // get its message data
                motor->write(msg);
                
                // write the message to the correct bus
                m_busses[bus]->write(msg);

                break;
            }
            default: {
                Serial.printf("CANManager tried to write to a motor of invalid type: %d\n", motor->get_controller_type());
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

void CANManager::send_to_comms(){
    for(const auto& [name, motor] : m_motor_name_map) {
        Comms::Sendable<MotorStateData> motor_state_sendable;
        MotorState state = motor->get_state();
        motor_state_sendable.data.motor_name = name;
        motor_state_sendable.data.torque = state.torque;
        motor_state_sendable.data.commanded_torque = motor->get_commanded_motor_torque();
        motor_state_sendable.data.speed = state.speed;
        motor_state_sendable.data.position = state.position;
        motor_state_sendable.data.temperature = state.temperature;
        
        motor_state_sendable.send_to_comms();
    }
}

void CANManager::issue_safety_mode() {
    // for each motor, cant be const
    for (auto& [name, motor] : m_motor_name_map) {
        motor->zero_motor();
    }

    // write the zero torque commands to the bus
    write();
}

void CANManager::write_motor_torque_by_name(Cfg::MotorName motor_name, float torque) {
    safety::assert_or_safety_procedure(motor_name!= Cfg::MotorName::UnsetMotorName, 
                                        "CANManager: Requested write to an unset motor name");

    safety::assert_or_safety_procedure(!m_motor_name_map.count(motor_name) == 0,
                                        "CANManager: Requested write to an invalid motor name: %u", static_cast<uint32_t>(motor_name));


    m_motor_name_map[motor_name]->write_motor_torque(torque);

    #ifdef CAN_MANAGER_DEBUG
    Serial.printf("CANManager wrote to motor with name %u\n", static_cast<uint32_t>(motor_name));
    #endif
}

void CANManager::print_state() {
    // for each motor, print it's state
    for (const auto& [name, motor] : m_motor_name_map) {
        // print the motor state
        motor->print_state();
    }
}

void CANManager::print_motor_state_by_name(Cfg::MotorName motor_name) {
    safety::assert_or_safety_procedure(motor_name!= Cfg::MotorName::UnsetMotorName, 
                                        "CANManager: Requested print of an unset motor name");
    safety::assert_or_safety_procedure(!m_motor_name_map.count(motor_name) == 0,
                                        "CANManager: Requested print of an invalid motor name: %u", static_cast<uint32_t>(motor_name));

    // print the motor state
    m_motor_name_map[motor_name]->print_state();
}

std::shared_ptr<Motor> CANManager::get_motor_by_name(Cfg::MotorName motor_name) {
    safety::assert_or_safety_procedure(motor_name!= Cfg::MotorName::UnsetMotorName, 
                                        "CANManager: Requested get of an unset motor name");
    safety::assert_or_safety_procedure(!m_motor_name_map.count(motor_name) == 0,
                                        "CANManager: Requested get of an invalid motor name: %u", static_cast<uint32_t>(motor_name));

    return m_motor_name_map[motor_name];
}

MotorState CANManager::get_motor_state_by_name(Cfg::MotorName motor_name) const {
    safety::assert_or_safety_procedure(motor_name!= Cfg::MotorName::UnsetMotorName, 
                                        "CANManager: Requested get of an unset motor name");
    safety::assert_or_safety_procedure(!m_motor_name_map.count(motor_name) == 0,
                                        "CANManager: Requested get of an invalid motor name: %u", static_cast<uint32_t>(motor_name));

    // return the motor state
    return m_motor_name_map.at(motor_name)->get_state();
}

void CANManager::init_motors() {
    // all motors have been created, go through and verify we are getting data from them and call their init functions

    // for each motor, call it's init function, cant be const
    for (auto& [name, motor] : m_motor_name_map) {
        // call the motor's init function
        motor->init();
    }

    // issue a write command to the bus to push the init commands
    // this cant easily be placed in the same loop as the init functions since some motors dont allow single writes
    write();

    // wait for the motors to initialize or timeout

    // maintain a list of motors that have been initialized, indexed by motor name
    std::set<Cfg::MotorName> initialized_motors;

    // only run the initialization for the timeout time
    uint32_t start_time = millis();

    while (millis() - start_time < m_motor_init_timeout) {
        // for each bus
        for (uint32_t bus = 0; bus < CAN_NUM_BUSSES; bus++) {
            CAN_message_t msg;

            // we want to read all the messages from this bus as there might be many queued up
            while (m_busses[bus]->read(msg)) {
                // try to distribute the message to the correct motor
                Cfg::MotorName recieving_motor_name = distribute_msg(msg);

                // no motor could handle the message so move on
                if (recieving_motor_name == Cfg::MotorName::UnsetMotorName) continue;
                // mark this motor as initialzied
                initialized_motors.insert(recieving_motor_name);
            }
        }
    }

    // print out any motors that failed to initialize using the motors_initialized array
    // this is not fatal but should be investigated
    for (const auto& [name, motor] : m_motor_name_map) {
        if (!initialized_motors.contains(name)) {
            Serial.printf("Warning: A motor failed to initialize! Motor info: ");
            motor->print_state();
        }
    }
}

Cfg::MotorName CANManager::distribute_msg(CAN_message_t& msg) {
    // for each motor, cant be const
    for (auto& [name, motor] : m_motor_name_map) {
        // if the motor can handle the message, give it to the motor
        if (motor->read(msg)) {
            return name;
        }
    }
    
    // no motors could handle the message
    return Cfg::MotorName::UnsetMotorName;
}
