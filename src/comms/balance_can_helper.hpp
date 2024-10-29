#ifndef BALANCE_CAN_HELPER
#define BALANCE_CAN_HELPER

// FlexCAN_T4 library
#include <FlexCAN_T4.h>

// LK TECH MG8016E-i6 V3 
// Docunebtation: http://en.lkmotor.cn/upload/20230321150047f.pdf
// CAN PROTOCOL: http://en.lkmotor.cn/upload/20230706100134f.pdf  

constexpr uint16_t CAN_3 = 2;

#define MG8016
#define NUM_CAN_BUSES      1 // 1 can for the MG8016 motors
#define NUM_MOTORS_PER_BUS 4 // 4 motors per can
#define NUM_MOTORS         (NUM_CAN_BUSES * NUM_MOTORS_PER_BUS)
#define NUM_MESSAGE_IDS    4 // each message ID controls one motor: they are 0x140 + ID(1~32)
#define CAN_MESSAGE_SIZE   8 // 8 uint8_t's per message buffer












/**Important!!!!!!!!!!!!!!!!!*/

/** Every functions with "set" will write to the DOM, multiple writes will affect thechip life. frequent use is not recommended  */










struct MG8016_Data{
    int8_t temperature; // 1℃/LSB
    uint16_t voltage; // 0.1V/LSB
    uint8_t error_state; // the bits represent different motor states
    /// @note Take errorState each bit specific state sheet for reference.
    uint8_t PID[6]; // 0: 
    int16_t toque_current; // range is -2048~2048, corresponding actual current is -33A~33A
    int16_t speed; // 1dps/LSB
    uint16_t encoder; // 14bit encoder range is 0~16383
    int16_t A_phase_current; // 1A/64LSB.
    int16_t B_phase_current; // 1A/64LSB.
    int16_t C_phase_current; // 1A/64LSB.
};


class MG8016_can{
private:
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> m_can3;
    CAN_message_t messages[NUM_MESSAGE_IDS];
    /// @brief Clear all messages
    void zero_all();
    /// @brief Clear messages for a single message id
    /// @param id the message id or the one motor id (1,2,3,4)
    void zero_id(uint8_t id);
public:
    MG8016_Data motor_data[NUM_MOTORS];
    void MG8016_init();
    /// @brief Turn off all MG8016 motors (No.1)
    void MG8016_off_all();
    /// @brief Turn on all MG8016 motors (No.2)
    void MG8016_on_all();
    /// @brief stop the motor but don't clear the motor state (No.3)
    /// @param id the one motor id (1,2,3,4)
    void MG8016_stop(uint8_t id);
    /// @brief control the open loop voltage (No.4)
    /// @param id the one motor id (1,2,3,4)
    /// @param voltage int16_t, range is -850~850
    void MG8016_openloop_contorl(uint8_t id, int16_t voltage);
    /// @brief control the torque current output (No.5)
    /// @param id the one motor id (1,2,3,4)
    /// @param current int16_t, range is -2048~2048 (-33A~33A)
    void MG8016_torque_closedloop_control(uint8_t id, int16_t current);
    /// @brief control motor speed (No.6)
    /// @param id the one motor id (1,2,3,4)
    /// @param speed int32_t, 0.01dps/LSB
    void MG8016_speed_closedloop_control(uint8_t id, int32_t speed);
    /// @brief control the position of the motor(Multi turn angle) with max speed (No.7)
    /// @param id the one motor id (1,2,3,4)
    /// @param angle int32_t, 0.01degree/LSB 
    /// @note motor spindirection is determined by the difference between the target position and the current position.
    void MG8016_multiloop_angle_control_max_speed(uint8_t id, int32_t angle);
    /// @brief control the position of the motor(Multi turn angle) with limited speed (No.8)
    /// @param id the one motor id (1,2,3,4)
    /// @param angle int32_t, 0.01degree/LSB 
    /// @param speed_limit uint16_t, 1dps/LSB
    /// @note motor spindirection is determined by the difference between the target position and the current position.
    void MG8016_multiloop_angle_control_limited_speed(uint8_t id, int32_t angle, uint16_t speed_limit);
    /// @brief control the position of the motor(Single turn angle) with max speed (No.9)
    /// @param id the one motor id (1,2,3,4)
    /// @param spin_direction uint8_t, (0x00: clockwise, 0x01: counterclockwise)
    /// @param angle uint32_t, 0.01degree/LSB 
    void MG8016_sigleloop_angle_control_max_speed(uint8_t id, uint8_t spin_direction, int32_t angle);
    /// @brief control the position of the motor(Single turn angle) with limited speed (No.10)
    /// @param id the one motor id (1,2,3,4)
    /// @param spin_direction uint8_t, (0x00: clockwise, 0x01: counterclockwise)
    /// @param angle uint32_t, 0.01degree/LSB 
    /// @param speed_limit uint16_t, 1dps/LSB
    void MG8016_singleloop_angle_control_limited_speed(uint8_t id, uint8_t spin_direction, int32_t angle);
    /// @brief control the increment angle of the motor with max speed (No.11)
    /// @param id the one motor id (1,2,3,4)
    /// @param angle int32_t, 0.01degree/LSB
    /// note motor spin direction is determined by the symbol of parameter. 
    void MG8016_increment_angle_control_max_speed(uint8_t id, int32_t angle);
    /// @brief control the increment angle of the motor with limited speed (No.12)
    /// @param id the one motor id (1,2,3,4)
    /// @param angle uint32_t, 0.01degree/LSB 
    /// @param speed_limit uint16_t, 1dps/LSB
    void MG8016_increment_angle_control_limited_speed(uint8_t id, int32_t angle, uint16_t speed_limit);
    /// @brief read current motor PID parameters (No.13)
    /// @param id the one motor id (1,2,3,4)
    /// @note the Data will go to the motor_data[id - 1].PID array
    void MG8016_read_PID_parameter(uint8_t id);
    /// @brief write PID parameters to RAM which will be deleted when power off (No.14)
    /// @param id the one motor id (1,2,3,4)
    /// @note the PID will get from the motor_data[id - 1].PID array
    void MG8016_write_PID_RAM(uint8_t id);
    /// @brief write PID parameters to ROM which will keep when power off (No.15)
    /// @param id the one motor id (1,2,3,4)
    /// @note the PID will get from the motor_data[id - 1].PID array
    void MG8016_set_PID_ROM(uint8_t id);
    /// @brief read motor acceleration parameter (No.16)
    /// @param id the one motor id (1,2,3,4)
    /// @return acceleration, int32_t, 1dps/s
    int32_t MG8016_read_acceleration(uint8_t id);
    /// @brief write acceleration to RAM (No.17)
    /// @param id the one motor id (1,2,3,4)
    /// @param accel int32_t, 1dps/s
    void MG8016_write_acceleration_RAM(uint8_t id, int32_t accel);
    /// @brief read current encoder position (No.18)
    /// @param id the one motor id (1,2,3,4)
    /// @param which 0~2 for encoder, encoder raw, and encoder offset
    /// @return encoder(uint16_t, 14 bit encoder range is 0~16383) is encoder raw value minus encoder offset value.
    /// @return encoderRaw(uint16_t,14 bit encoder range is 0~16383)
    /// @return encoderOffset(uint16_t,14 bit encoder range is 0~16383). This point is the initial zero position of the motor.
    uint16_t MG8016_read_encoder(uint8_t id, uint8_t which);
    /// @brief set encoder offset to ROM as the initial position (zero point) (No.19)
    /// @param id the one motor id (1,2,3,4)
    /// @param encoder_offset uint16_t,14bit encoder range is 0~16383.
    void MG8016_set_encoder_offset(uint8_t id, uint16_t encoder_offset);
    /// @brief Write motor encoder current position to ROM as the initial position (zero point) (No.20)
    /// @param id the one motor id (1,2,3,4) 
    /// @note The command will be valid only after reset power
    void MG8016_set_encoder_current(uint8_t id);
    /// @brief Read the muilt angle (No.21)
    /// @param id the one motor id (1,2,3,4) 
    /// @return motorAngle, int64_t, 0.01°/LSB.
    int64_t MG8016_read_multi_angle(uint8_t id);
    /// @brief Read the single angle (No.22)
    /// @param id the one motor id (1,2,3,4) 
    /// @return circleAngle, uint32_t. 0.01°/LSB (value range 0~35999)
    uint32_t MG8016_read_single_angle(uint8_t id);
    /// @brief clear motor multi turn and single turn data and set current position as motor zeropoint. It’s invalid when power off (No.23)
    /// @param id the one motor id (1,2,3,4) 
    void MG8016_clear_angle_loop(uint8_t id);
    /// @brief read current motor temperature, voltage and error state (No.24)
    /// @param id the one motor id (1,2,3,4) 
    /// @note Data goes to motor_data[id - 1]
    void MG8016_read_motor_state_1(uint8_t id);
    /// @brief clear motor current error state (No.25)
    /// @param id the one motor id (1,2,3,4) 
    /// @note 1. Error can’t be cleared unless the motor state is back to normal
    /// @note 2. Take errorState each bit specific state sheet for reference
    /// @author IDK WTF is this
    void MG8016_clear_error_state(uint8_t id);
    /// @brief read current temperature,voltage,speed and encoder position
    /// @param id the one motor id (1,2,3,4) 
    /// @note Data goes to motor_data[id - 1]
    void MG8016_read_motor_state_2(uint8_t id);
    /// @brief read current motor temperature and phase current data
    /// @param id the one motor id (1,2,3,4)
    /// @note Data goes to motor_data[id - 1]
    void MG8016_read_motor_state_3(uint8_t id);
}
#endif