#include "motor.hpp"

// motor spec sheets: 
// https://steadywin.cn/en/pd.jsp?id=15&fromColId=0#_pp=0_752_3
// https://steadywin.cn/en/pd.jsp?id=116&fromColId=0#_pp=0_752_3
// https://steadywin.cn/en/pd.jsp?id=11&fromColId=0#_pp=0_752_3
// https://steadywin.cn/en/pd.jsp?id=130&fromColId=0#_pp=0_752_3
// motor driver protocol spec: https://14180476.s21i.faiusr.com/61/ABUIABA9GAAgqPvVqgYo-uLA7Ac.pdf?v=1708485774

/// @brief Motor driver for the GIM motor
class GIM : public Motor {
public:
    /// @brief Deleted default constructor, must explicitly construct this object. Incomplete objects are not allowed
    GIM() = delete;

    /// @brief Main constructor. Defines the controller type, global ID, id, and can bus
    /// @param gid The global ID, not the per-bus motor ID
    /// @param id The per-bus motor ID. This is 1-indexed
    /// @param bus_id The CAN bus index/ID
    GIM(uint32_t gid, uint32_t id, uint8_t bus_id, MotorType motor_type)
        : Motor(MotorControllerType::MG8016, gid, id, bus_id, motor_type) {

        switch (motor_type) {
        case MotorType::GIM3505: {
            torque_constant = 0.52f;
            gear_ratio = 8.0f; // 8:1
            max_torque = 1.27f;
            break;
        }
        case MotorType::GIM4310: {
            torque_constant = 3.46f;
            gear_ratio = 36.0f; // 36:1
            max_torque = 20.16f;
            break;
        }
        case MotorType::GIM6010: {
            torque_constant = 0.47f;
            gear_ratio = 8.0f; // 8:1
            max_torque = 11.0f;

            break;
        }
        case MotorType::GIM8108: {
            // TODO: this motor has two versions and we need to know which one we have. 
            // torque constant is either 1.83 or 0.96.
            torque_constant = 0.0f;
            gear_ratio = 9.0f; // 9:1
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

    /// @brief The motor's gear ratio (determined by "motor_type" in the constructor)
    float gear_ratio;

    /// @brief The motor's torque constant in Nm/A (determined by "motor_type" in the constructor)
    float torque_constant;

    /// @brief The motor's maximum torque in Nm (determined by "motor_type" in the constructor)
    float max_torque;

public:
    /// @brief Initialize the motor by verifying it is on
    void init() override;

    int read(CAN_message_t& msg) override;

    int write(CAN_message_t& msg) const override;

    void zero_motor() override;

    void write_motor_torque(float torque);

public:
    void write_motor_on();

    void write_motor_off();

    void write_motor_stop();


private:
    const uint32_t m_base_id = 0x0;

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
    void create_cmd_calibration(uint8_t buf[8], uint8_t calibration_type);

    /// @brief Create an update firmware command
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