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
        : Motor(MotorControllerType::MG8016_CONTROLLER, gid, id, bus_id, motor_type) {
        
        switch(motor_type) {
            case MotorType::GIM3505: {
                torque_constant = 0.52;
                gear_ratio = 8.0; // 8:1
                break;
            }
            case MotorType::GIM4310: {
                torque_constant = 3,46;
                gear_ratio = 36.0; // 36:1
                break;
            }
            case MotorType::GIM6010: {
                torque_constant = 0.47;
                gear_ratio = 8.0; // 8:1
                break;
            }
            case MotorType::GIM8108: {
                // TODO: this motor has two versions and we need to know which one we have. 
                // torque constant is either 1.83 or 0.96.
                gear_ratio = 9.0; // 9:1
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
    double gear_ratio;

    /// @brief The motor's torque constant in Nm/A (determined by "motor_type" in the constructor)
    double torque_constant;

public:
    /// @brief Initialize the motor by verifying it is on
    void init() override;

    int read(CAN_message_t& msg) override;

    int write(CAN_message_t& msg) const override;

    void zero_motor() override;

    void write_motor_torque(float torque);

    void print_state() const override;
public:
    void write_motor_on();

    void write_motor_off();

    void write_motor_stop();


private:
    const uint32_t m_base_id = 0x0;

    void create_cmd_retrieve_configuration(uint8_t buf[8], uint8_t conf_type, uint8_t conf_id);

    void create_cmd_motor_on(uint8_t buf[8]);

    void create_cmd_motor_off(uint8_t buf[8]);

    void create_cmd_motor_stop(uint8_t buf[8]);

    void create_cmd_torque_control(uint8_t buf[8], float torque, uint32_t duration);

    void create_cmd_speed_control(uint8_t buf[8], float speed, uint32_t duration);

    static constexpr uint8_t CMD_RETRIEVE_CONFIGURATION = 0x84;

    static constexpr uint8_t CMD_MOTOR_ON = 0x91;

    static constexpr uint8_t CMD_MOTOR_OFF = 0x92;

    static constexpr uint8_t CMD_TORQUE_CONTROL = 0x93;

    static constexpr uint8_t CMD_SPEED_CONTROL = 0x94;

    static constexpr uint8_t CMD_MOTOR_STOP = 0x97;

    static constexpr uint8_t CMD_GET_VERSION = 0xB1;

    static constexpr uint8_t CMD_GET_FAULT = 0xB2;
};