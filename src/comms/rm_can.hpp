#ifndef CAN_MANAGER_HPP
#define CAN_MANAGER_HPP

// FlexCAN_T4 library
#include <FlexCAN_T4.h>

// C620 Brushless DC Motor Speed Controller
// Documentation: https://rm-static.djicdn.com/tem/17348/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.01.pdf 

// FlexCAN_T4
// Documentation: https://github.com/tonton81/FlexCAN_T4

constexpr uint16_t CAN_1 = 0;             // CAN 1 (indexable value)
constexpr uint16_t CAN_2 = 1;             // CAN 2 (indexable value)

#define NUM_CAN_BUSES      2 // 2 cans per robot
#define NUM_MOTORS_PER_BUS 8 // 8 motors per can
#define NUM_MOTORS         (NUM_CAN_BUSES * NUM_MOTORS_PER_BUS)
#define NUM_MESSAGE_IDS    3 // 3 messages per can: 0x200, 0x1ff, 0x2ff
#define CAN_MESSAGE_SIZE   8 // 8 uint8_t's per message buffer

#define C610   0
#define C620   1
#define GM6020 2

#define C610_OUTPUT_SCALE  10000
#define C620_OUTPUT_SCALE  16384
#define GM6020_OUTPUT_SCALE 30000

/// @brief Returns a 2-byte value given 2 1-byte values
/// @param highByte higher order byte
/// @param lowByte lower order byte
/// @return Constructed 2-byte value in form [high, low]
uint16_t combine_bytes(uint8_t high, uint8_t low);


/// @brief Simple enum to specify types of values able to be read from motors
enum MotorAttribute {
    ANGLE, SPEED, TORQUE, TEMP
};

/// @brief The purpose of this struct is to be able to pass updated CAN data around without having to pass a rm_CAN object pointer which can be finnicky.
struct CANData {
    /// @brief actual stored motor data to be sent around
    uint8_t data[NUM_CAN_BUSES][NUM_MOTORS_PER_BUS][CAN_MESSAGE_SIZE];

    /// @brief Reads and returns value from input array of specified motor
    /// @param canID ID of the CAN which the motor is on, expects indexable ID value
    /// @param motorID ID of the individual motor, expects indexable ID value
    /// @param valueType MotorAttribute enum for what type of value requested
    /// @return Requested value
    int get_motor_attribute(uint16_t canID, uint16_t motorID, MotorAttribute valueType) {
        // return correct value depending on valueType enum
        switch (valueType) {
        case MotorAttribute::ANGLE:
            return (uint16_t)(combine_bytes(data[canID][motorID - 1][0], data[canID][motorID - 1][1]));
            break;
        case MotorAttribute::SPEED:
            return (int16_t)(combine_bytes(data[canID][motorID - 1][2], data[canID][motorID - 1][3]));
            break;
        case MotorAttribute::TORQUE:
            return (int16_t)(combine_bytes(data[canID][motorID - 1][4], data[canID][motorID - 1][5]));
            break;
        case MotorAttribute::TEMP:
            return (uint16_t)(data[canID][motorID - 1][6]);
            break;

        default:
            return 0;
            break;
        }
    }
};

/// @brief Manages both CANs on the robot. Able to read from and write to individual or multiple motors on the CANs.
class rm_CAN {
public:
    /// @brief Empty constructor (implementation weirdly needs to be in the .cpp)
    rm_CAN();

    /// @brief Initializes and zeros CANs
    void init();

    /// @brief Reads all inbound CAN packets and fills the input array accordingly
    void read();

    /// @brief Writes current values from output array to the CANs
    /// @return True or false depending if the operation was successful
    /// @note Does issue a Write command to the CANs
    uint8_t write();

public:
    /// @brief Sets the buffer values in output array to 0
    /// @note Does not issue a Write command to the CANs
    void zero_motors();

    /// @brief Sets output array to 0 and writes 0s to the CANs
    /// @note Does issue a Write command to the CANs
    void zero();

    /// @brief Sets the output array message for the corresponding motor
    /// @note Does not issue a Write command to the CANs
    /// @param canID ID of the CAN which the motor is on, expects indexable ID value
    /// @param motorID ID of the individual motor, expects indexable ID value
    /// @param value A value in the range of [-16385, 16384] which maps to [-20A, 20A]
    void write_motor(uint16_t canID, uint16_t motorID, int32_t value);

    /// @brief Sets the output array message for the corresponding motor, with a normalized value input
    /// @note Does not issue a Write command to the CANs
    /// @param canID ID of the CAN which the motor is on, expects indexable ID value
    /// @param motorID ID of the individual motor, expects indexable ID value
    /// @param controllerType type of motor controller that this motor uses
    /// @param value A value in the range of [-1.0, 1.0]
    void write_motor_norm(uint16_t canID, uint16_t motorID, uint8_t controllerType, float value);

    /// @brief Reads and returns value from input array of specified motor
    /// @param canID ID of the CAN which the motor is on, expects indexable ID value
    /// @param motorID ID of the individual motor, expects indexable ID value
    /// @param valueType MotorAttribute enum for what type of value requested
    /// @return Requested value
    int get_motor_attribute(uint16_t canID, uint16_t motorID, MotorAttribute valueType);

    /// @brief Print off all values associated with specified motor
    /// @param canID ID of the CAN which the motor is on, expects indexable ID value
    /// @param motorID ID of the individual motor, expects indexable ID value
    /// @param showFullHex Show the entire buffer in hex, rather than decimal values (defaults to false)
    void print_motor(uint16_t canID, uint16_t motorID, bool showFullHex = false);

    /// @brief Print off all motors associated with specified CAN
    /// @param canID ID of the CAN which the motor is on, expects indexable ID value
    /// @param showFullHex Show the entire buffer in hex, rather than decimal values (defaults to false)
    void print_can(uint16_t canID, bool showFullHex = false);

    /// @brief Print off the values of the output array
    void print_output();

    /// @brief get can data pointer from can
    /// @return can data pointer
    CANData* get_data() { return &m_input; }

private:
    /// @brief CAN 1 object
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> m_can1;
    /// @brief CAN 2 object
    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> m_can2;

    /// @brief Output array in the format of CAN_message_t's
    CAN_message_t m_output[NUM_CAN_BUSES][NUM_MESSAGE_IDS];
    /// @brief Input array in the format of uint8_t's
    CANData m_input;

};

#endif // CAN_MANAGER_HPP
