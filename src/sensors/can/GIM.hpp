#include "motor.hpp"



/// @brief Motor driver for the GIM motor
/// @note GIM8108 docs (SDC103): https://steadywin.cn/en/pd.jsp?id=15&fromColId=0#_pp=0_752_3
/// @note GIM4310 docs         : https://steadywin.cn/en/pd.jsp?id=11&fromColId=0#_pp=0_752_3
/// @note GIM3505 docs         : https://steadywin.cn/en/pd.jsp?id=130&fromColId=0#_pp=0_752_3
/// @note motor driver protocol: https://14180476.s21i.faiusr.com/61/ABUIABA9GAAgqPvVqgYo-uLA7Ac.pdf?v=1708485774
class GIM : public Motor {
public:
    /// @brief Deleted default constructor, must explicitly construct this object. Incomplete objects are not allowed
    GIM() = delete;

    /// @brief Main constructor. Defines the controller type, global ID, id, and can bus
    /// @param gid The global ID, not the per-bus motor ID
    /// @param id The per-bus motor ID. This is 1-indexed
    /// @param bus_id The CAN bus index/ID
    /// @param motor_type The motor type, used for GIM to determine gear ratio, max torque, max speed, and torque constant
    GIM(uint32_t gid, uint32_t id, uint8_t bus_id, MotorType motor_type)
        : Motor(MotorControllerType::MG8016, gid, id, bus_id, motor_type) {

        switch (motor_type) {
        case MotorType::GIM3505: {
            m_torque_constant = 0.52f;
            m_gear_ratio = 8.0f; // 8:1
            m_max_torque = 1.27f; // Nm
            m_max_speed = 225.0f; // RPM
            break;
        }
        case MotorType::GIM4310: {
            m_torque_constant = 3.46f;
            m_gear_ratio = 36.0f; // 36:1
            m_max_torque = 20.16f; // Nm
            m_max_speed = 63.0f; // RPM
            break;
        }
        case MotorType::GIM8108: {
            m_torque_constant = 1.83f;
            m_gear_ratio = 9.0f; // 9:1
            m_max_torque = 27.38; // Nm
            m_max_speed = 242.0f; // RPM
            break;
        }
        default: {
            Serial.printf("GIM motor type not recognized: %d\n", motor_type);
            break;
        }
        }
    }

    /// @brief Destructor, does nothing
    ~GIM() override { }


public:
    /// @brief Initialize the motor by verifying it is on
    void init() override;

    /// @brief Common read command.
    /// @param msg The message buffer to fill data into
    /// @return 0 on failure, 1 on success
    int read(CAN_message_t& msg) override;

    /// @brief Common write command. This fills the given message if successful. This is used to compile data to be sent over the CAN line
    /// @param msg The message buffer to fill data into
    /// @return The index in the buffer where the motor data was written. The MG8016E-i6 driver will always write 8 bytes so this will always return 0
    /// @note Does not issue a CAN command over the bus
    /// @note Return value is mostly useless for this motor, it will always be 0
    int write(CAN_message_t& msg) const override;

    /// @brief Zero the motor. This is a safety function to ensure the motor is not actively driving
    /// @note Does not issue a CAN command over the bus
    void zero_motor() override;

    /// @brief Write the motor's torque
    /// @param torque Torque from -1.0f to 1.0f. This gets clamped into this range and then scaled to the motor type's max torque.
    void write_motor_torque(float torque);

    /// @brief Write the motor's speed
    /// @param speed Speed from -1.0f to 1.0f. This gets clamped into this range and then scaled to the individual motor type's max speed.
    void write_motor_speed(float speed);

    /// @brief Write the motor's position
    /// @param position Position in radians
    void write_motor_position(float position);

public:
    /// @brief Turn on the motor
    void write_motor_on();

    /// @brief Turn off the motor and clear its state
    void write_motor_off();

    /// @brief Stop the motor but don't clear its state
    void write_motor_stop();


private:
    /// @brief The motor's gear ratio (determined by "motor_type" switch in the constructor)
    float m_gear_ratio = 0.0f;

    /// @brief The motor's torque constant in Nm/A (determined by "motor_type" switch in the constructor)
    float m_torque_constant = 0.0f;

    /// @brief The motor's maximum torque in Nm (determined by "motor_type" switch in the constructor)
    float m_max_torque = 0.0f;

    /// @brief The motor's maximum speed in RPM (determined by "motor_type" switch in the constructor)
    float m_max_speed = 0.0f;

    /// @brief Creates a command to reset the motor's configuration
    /// @param buf Output buffer to write command to
    void create_cmd_reset_configuration(uint8_t buf[8]);

    /// @brief Creates a command to refresh the motor's configuration
    /// @param buf Output buffer to write command to
    void create_cmd_refresh_configuration(uint8_t buf[8]);

    /// @brief Creates a command to modify the configuration
    /// @param buf Output buffer to write command to
    /// @param conf_type 1 byte. 0x00 indicates data is a 32-bit signed INTEGER. 0x01 indicates that data is a 32-bit signed FLOAT. 
    /// @param conf_id 1 byte specifying which configuration to modify. these are found on page number 7 of https://14180476.s21i.faiusr.com/61/ABUIABA9GAAgqPvVqgYo-uLA7Ac.pdf?v=1708485774
    /// @param data 4 bytes of data to write to the configuration.
    void create_cmd_modify_configuration(uint8_t buf[8], uint8_t conf_type, uint8_t conf_id, uint32_t data);

    /// @brief Creates a command to retrieve the motor's configuration
    /// @param buf Output buffer to write command to
    /// @param conf_type 1 byte. 0x00 indicates data is a 32-bit signed INTEGER. 0x01 indicates that data is a 32-bit signed FLOAT.
    /// @param conf_id 1 byte specifying which configuration to retrieve.
    void create_cmd_retrieve_configuration(uint8_t buf[8], uint8_t conf_type, uint8_t conf_id);

    /// @brief Creates a command to start the motor
    /// @param buf Output buffer to write command to
    void create_cmd_start_motor(uint8_t buf[8]);

    /// @brief Creates a command to stop the motor
    /// @param buf Output buffer to write command to
    void create_cmd_stop_motor(uint8_t buf[8]);

    /// @brief Create a torque control command
    /// @param buf Output buffer to write command to
    /// @param torque Torque in unit of Nm
    /// @param duration Torque control execution time in ms.
    void create_cmd_torque_control(uint8_t buf[8], float torque, uint32_t duration);

    /// @brief Create a speed control command
    /// @param buf Output buffer to write command to
    /// @param speed Speed in unit of RPM.
    /// @param duration Speed control execution time in ms.
    void create_cmd_speed_control(uint8_t buf[8], float speed, uint32_t duration);

    /// @brief Create a position control command
    /// @param buf Output buffer to write command to
    /// @param position Target position in unit of RAD
    /// @param duration Position control execution time in ms.
    void create_cmd_position_control(uint8_t buf[8], float position, uint32_t duration);

    /// @brief Create a stop control command
    /// @param buf Output buffer to write command to
    void create_cmd_stop_control(uint8_t buf[8]);

    /// @brief Create a modify parameter command
    /// @param buf Output buffer to write command to
    /// @param param_id 1 byte parameter id to modify (i.e. 0x00, 0x01, ...)
    /// @param data 4 bytes of data to write to the parameter
    void create_cmd_modify_parameter(uint8_t buf[8], uint8_t param_id, uint32_t data);

    /// @brief Create a retrieve parameter command
    /// @param buf Output buffer to write command to
    /// @param param_id 1 byte parameter id to retrieve (i.e. 0x00, 0x01, ...)
    void create_cmd_retrieve_parameter(uint8_t buf[8], uint8_t param_id);

    /// @brief Create a get version command
    /// @param buf Output buffer to write command to
    void create_cmd_get_version(uint8_t buf[8]);

    /// @brief Create a get fault command
    /// @param buf Output buffer to write command to
    void create_cmd_get_fault(uint8_t buf[8]);

    /// @brief Create an acknowledge fault command
    /// @param buf Output buffer to write command to
    void create_cmd_acknowledge_fault(uint8_t buf[8]);

    /// @brief Create a retrieve indicator command
    /// @param buf Output buffer to write command to
    /// @param indicator_id 1 byte indicator id to retrieve (i.e. 0x00, 0x01, ...)
    void create_cmd_retrieve_indicator(uint8_t buf[8], uint8_t indicator_id);

    /// @brief Create a calibration command
    /// @param buf Output buffer to write command to
    /// @param calibration_type 1 byte calibration type, 0x00 phase order calibration, 0x01 encoder calibration 
    void create_cmd_calibration(uint8_t buf[8], uint8_t calibration_type);

    /// @brief Create an update firmware command
    /// @param buf Output buffer to write command to
    /// @param indicator_id the documentation doesn't say what this is for. niche command anyways.
    void create_cmd_update_firmware(uint8_t buf[8], uint8_t indicator_id);

    // Command byte definitions:

    // configuration commands

    /// @brief Reset configuration command byte
    static constexpr uint8_t CMD_RESET_CONFIGURATION = 0x81;
    /// @brief Refresh configuration command byte
    static constexpr uint8_t CMD_REFRESH_CONFIGURATION = 0x82;
    /// @brief Modify configuration command byte
    static constexpr uint8_t CMD_MODIFY_CONFIGURATION = 0x83;
    /// @brief Retrieve configuration command byte
    static constexpr uint8_t CMD_RETRIEVE_CONFIGURATION = 0x84;


    /// @brief Start motor command byte
    static constexpr uint8_t CMD_START_MOTOR = 0x91;
    /// @brief Stop motor command byte
    static constexpr uint8_t CMD_STOP_MOTOR = 0x92;
    /// @brief Torque control command byte
    static constexpr uint8_t CMD_TORQUE_CONTROL = 0x93;
    /// @brief Speed control command byte
    static constexpr uint8_t CMD_SPEED_CONTROL = 0x94;
    /// @brief Position control command byte
    static constexpr uint8_t CMD_POSITION_CONTROL = 0x95;

    // note: although a PTS (position,torque,speed) control command 0x96 is mentioned in the docs, it is marked unsupported and has no usage details.

    /// @brief Stop control command byte
    static constexpr uint8_t CMD_STOP_CONTROL = 0x97;

    /// @brief Modify parameter command byte
    static constexpr uint8_t CMD_MODIFY_PARAMETER = 0xA1;
    /// @brief Retrieve parameter command byte
    static constexpr uint8_t CMD_RETRIEVE_PARAMETER = 0xA2;

    /// @brief Get version command byte
    static constexpr uint8_t CMD_GET_VERSION = 0xB1;
    /// @brief Get fault command byte
    static constexpr uint8_t CMD_GET_FAULT = 0xB2;
    /// @brief Aknowledge fault command byte
    static constexpr uint8_t CMD_ACKNOWLEDGE_FAULT = 0xB3;
    /// @brief Retrieve indicator command byte
    static constexpr uint8_t CMD_RETRIEVE_INDICATOR = 0xB4;

    /// @brief Calibration command byte
    static constexpr uint8_t CMD_CALIBRATION = 0xB5;

    /// @brief update firmware command byte
    static constexpr uint8_t CMD_UPDATE_FIRMWARE = 0xC1;
};