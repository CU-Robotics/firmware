#pragma once
#include "Arduino.h"
#include "comms/config_data/motor.hpp"
#include "comms/config_data/sensor.hpp"
#include "utils/safety.hpp"
#include <array>
#include <memory>
#include <type_traits>

/* These adapters replace device I/O only. All estimation/governance/PID and
 * configured-state validation code is compiled directly from production. */
class Motor {
public:
    struct State { float speed = 0; };
    State state;
    float torque = 0;
    const State& get_state() const { return state; }
    void write_motor_torque(float value) { torque = std::clamp(value, -1.0f, 1.0f); }
};
class CANManager {
public:
    std::array<std::shared_ptr<Motor>, static_cast<size_t>(Cfg::MotorName::MotorNameCount)> motors;
    CANManager() { for (auto& motor : motors) motor = std::make_shared<Motor>(); }
    std::shared_ptr<Motor> get_motor_by_name(Cfg::MotorName name) {
        const auto index = static_cast<size_t>(name);
        safety::assert_or_safety_procedure(index > 0 && index < motors.size(), "Invalid simulated motor name");
        return motors[index];
    }
    void zero() { for (auto& motor : motors) motor->torque = 0; }
};
class BuffEncoder {
public:
    float angle = 0;
    float get_angle() const { return angle; }
};
class RevEncoder {};
class ICM20649 {
public:
    float gyro[3] = {};
    float get_gyro_X() const { return gyro[0]; }
    float get_gyro_Y() const { return gyro[1]; }
    float get_gyro_Z() const { return gyro[2]; }
};
class SensorManager {
public:
    std::shared_ptr<BuffEncoder> yaw = std::make_shared<BuffEncoder>();
    std::shared_ptr<BuffEncoder> pitch = std::make_shared<BuffEncoder>();
    std::shared_ptr<ICM20649> imu = std::make_shared<ICM20649>();
    template<class T> std::shared_ptr<T> get_sensor_by_name(Cfg::SensorName name) {
        if constexpr (std::is_same_v<T, BuffEncoder>) {
            if (name == Cfg::SensorName::YawBuffEncoder) return yaw;
            if (name == Cfg::SensorName::PitchBuffEncoder) return pitch;
        } else if constexpr (std::is_same_v<T, ICM20649>) {
            if (name == Cfg::SensorName::YawIcmImu) return imu;
        }
        safety::safety_procedure("Unsupported simulated sensor");
    }
};
struct SimRefSystem {
    struct Data {
        struct PowerHeat { float buffer_energy = 0; } robot_power_heat;
        struct LaunchingStatus { float initial_speed = 0; } launching_status;
    } ref_data;
};
inline SimRefSystem ref;
enum class Subsystem { ESTIMATOR, Controls };
struct SimSystemLog {
    template<class... Args> void info(Subsystem, const char* format, Args... args) { std::fprintf(stderr, format, args...); }
    template<class... Args> void warn(Subsystem, const char* format, Args... args) { std::fprintf(stderr, format, args...); }
    template<class... Args> void error(Subsystem, const char* format, Args... args) { std::fprintf(stderr, format, args...); }
};
inline SimSystemLog SystemLog;
struct SimSafetyState {
    bool active = false;
    bool is_safety_mode_active() const { return active; }
};
inline thread_local SimSafetyState safety_state;
