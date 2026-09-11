#include "can_manager.hpp"

// driver includes are here not in header since they're only needed in the implementation
#include "utils/safety.hpp"
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

/// @brief FlexCAN register base address of each bus, for reading its flags directly
static const uint32_t CAN_BUS_BASE[CAN_NUM_BUSSES] = { CAN1, CAN2, CAN3 };

/// @brief Short motor name for the feedback statistics prints
/// @param name The motor name
/// @return A printable name
static const char* motor_name_to_string(Cfg::MotorName name) {
    switch (name) {
    case Cfg::MotorName::Chassis1:          return "Chassis1";
    case Cfg::MotorName::Chassis2:          return "Chassis2";
    case Cfg::MotorName::Chassis3:          return "Chassis3";
    case Cfg::MotorName::Chassis4:          return "Chassis4";
    case Cfg::MotorName::Yaw1:              return "Yaw1";
    case Cfg::MotorName::Yaw2:              return "Yaw2";
    case Cfg::MotorName::Pitch1:            return "Pitch1";
    case Cfg::MotorName::Pitch2:            return "Pitch2";
    case Cfg::MotorName::Flywheel1:         return "Flywheel1";
    case Cfg::MotorName::Flywheel2:         return "Flywheel2";
    case Cfg::MotorName::Flywheel3:         return "Flywheel3";
    case Cfg::MotorName::Feeder:            return "Feeder";
    case Cfg::MotorName::UpperFeeder:       return "UpperFeeder";
    case Cfg::MotorName::LowerFeederClose:  return "LowerFeederClose";
    case Cfg::MotorName::LowerFeederFar:    return "LowerFeederFar";
    default:                                return "Unknown";
    }
}

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

    // start the first feedback statistics window after init so it doesn't include boot time
    m_feedback_stats_start_ms = millis();
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
    // count FIFO warnings/overflows from since the last read, before draining the FIFOs
    check_rx_fifo_flags();

    // for each bus
    for (uint32_t bus = 0; bus < CAN_NUM_BUSSES; bus++) {
        // we want to read all the messages from this bus as there might be many queued up
        CAN_message_t msg;
        while (true) {
            // FlexCAN_T4's read() picks the FIFO or the mailboxes at random and returns 0 if the one it picked is empty,
            // even when the FIFO still has frames. Only stop once the FIFO is empty too, or frames pile up and get dropped.
            if (!m_busses[bus]->read(msg)) {
                if (FLEXCANb_IFLAG1(CAN_BUS_BASE[bus]) & FLEXCAN_IFLAG1_BUF5I) continue;
                break;
            }

            m_bus_frames[bus]++;
            if (msg.flags.overrun) m_mailbox_overruns[bus]++;

            // distribute the message to the correct motor
            // if this fails, we've received a message that does not match any motor
            // how would this happen?
            Cfg::MotorName motor_name = distribute_msg(msg);
            if (motor_name == Cfg::MotorName::UnsetMotorName) {
                // - 1 on msg.bus to maintain bus IDs being 0-indexed
                Serial.printf("CANManager failed to distribute message with raw CAN ID: %.4x on bus: %x\n", msg.id, msg.bus - 1);
            } else {
                record_feedback_frame(motor_name, msg);
            }

            // the mailbox read only ever sets this flag, so clear it before msg is reused
            msg.flags.overrun = false;
        }
    }

    if (millis() - m_feedback_stats_start_ms >= m_feedback_stats_period_ms) {
        print_feedback_stats();
    }
}

void CANManager::record_feedback_frame(Cfg::MotorName motor_name, const CAN_message_t& msg) {
    uint32_t now_us = micros();
    FeedbackStats& stats = m_feedback_stats[motor_name];

    if (stats.last_frame_us != 0 && now_us - stats.last_frame_us > stats.max_gap_us) {
        stats.max_gap_us = now_us - stats.last_frame_us;
    }
    stats.last_frame_us = now_us;
    stats.can_id = msg.id;
    stats.frames++;
}

void CANManager::check_rx_fifo_flags() {
    // FlexCAN_T4 only clears the FIFO warning (bit 6) and overflow (bit 7) flags in its interrupt handler,
    // which we don't use, so read and clear them here. Writing a 1 clears a flag and leaves the others alone.
    for (uint32_t bus = 0; bus < CAN_NUM_BUSSES; bus++) {
        uint32_t fifo_flags = FLEXCANb_IFLAG1(CAN_BUS_BASE[bus]) & (FLEXCAN_IFLAG1_BUF6I | FLEXCAN_IFLAG1_BUF7I);
        if (fifo_flags & FLEXCAN_IFLAG1_BUF6I) m_fifo_warnings[bus]++;
        if (fifo_flags & FLEXCAN_IFLAG1_BUF7I) m_fifo_overflows[bus]++;
        FLEXCANb_IFLAG1(CAN_BUS_BASE[bus]) = fifo_flags;
    }
}

void CANManager::print_feedback_stats() {
    uint32_t now_ms = millis();
    uint32_t now_us = micros();
    uint32_t window_ms = now_ms - m_feedback_stats_start_ms;

    for (uint32_t bus = 0; bus < CAN_NUM_BUSSES; bus++) {
        bool bus_has_motors = false;
        for (const auto& [name, motor] : m_motor_name_map) {
            if (motor->get_bus_id() == bus) bus_has_motors = true;
        }

        if (bus_has_motors || m_bus_frames[bus] > 0) {
            Serial.printf("CAN bus %lu (%lu ms): %lu frames/s, FIFO warn %lu, FIFO overflow %lu, MB overrun %lu |",
                          bus, window_ms, m_bus_frames[bus] * 1000 / window_ms,
                          m_fifo_warnings[bus], m_fifo_overflows[bus], m_mailbox_overruns[bus]);

            for (const auto& [name, motor] : m_motor_name_map) {
                if (motor->get_bus_id() != bus) continue;

                FeedbackStats& stats = m_feedback_stats[name];
                if (stats.last_frame_us == 0) {
                    Serial.printf(" %s id %lu: no frames yet |", motor_name_to_string(name), motor->get_id());
                    continue;
                }

                // include the time since the last frame so a motor that went quiet this window still shows a gap
                uint32_t gap_us = now_us - stats.last_frame_us;
                if (stats.max_gap_us > gap_us) gap_us = stats.max_gap_us;

                Serial.printf(" %s 0x%03lx: %lu/s gap %lums |", motor_name_to_string(name), stats.can_id,
                              stats.frames * 1000 / window_ms, gap_us / 1000);

                stats.frames = 0;
                stats.max_gap_us = 0;
            }
            Serial.printf("\n");
        }

        m_bus_frames[bus] = 0;
        m_mailbox_overruns[bus] = 0;
        m_fifo_warnings[bus] = 0;
        m_fifo_overflows[bus] = 0;
    }

    m_feedback_stats_start_ms = now_ms;
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
