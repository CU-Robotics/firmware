#include "bridge.h"
#include "controls/controller.hpp"
#include "controls/estimator.hpp"
#include "controls/reference_governor.hpp"
#include "hardware.hpp"
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#include <vector>

namespace {
constexpr Cfg::StateName state_names[5] = {Cfg::StateName::ChassisX, Cfg::StateName::ChassisY, Cfg::StateName::ChassisHeading, Cfg::StateName::GimbalYaw, Cfg::StateName::GimbalPitch};
constexpr Cfg::MotorName motor_names[4] = {Cfg::MotorName::Yaw1, Cfg::MotorName::Yaw2, Cfg::MotorName::Pitch1, Cfg::MotorName::Pitch2};
void error_text(char* out, uint32_t capacity, const char* text) { if (out && capacity) std::snprintf(out, capacity, "%s", text); }
bool finite_values(const float* values, size_t count) { for (size_t i = 0; i < count; ++i) if (!std::isfinite(values[i])) return false; return true; }
Cfg::StateLimit limits(const float* values) { return {{values[0], values[1]}, {values[2], values[3]}, {values[4], values[5]}}; }
std::vector<Cfg::State> states_from(const FirmwareSimConfig& config) {
    std::vector<Cfg::State> states(5);
    for (size_t i = 0; i < states.size(); ++i) {
        const auto& src = config.states[i]; auto& dst = states[i];
        dst.name = state_names[i]; dst.reference_limits = limits(src.reference_limits); dst.physical_limits = limits(src.physical_limits);
        dst.governor_type = static_cast<Cfg::StateOrder>(src.governor_type); dst.is_wrapping = src.is_wrapping;
        dst.max_controller_error = src.max_controller_error; dst.max_error_exceed_time_us = src.max_error_exceed_time_us;
    }
    return states;
}
Cfg::Controller controller_from(const FirmwareSimControllerConfig& src, bool yaw) {
    Cfg::Controller dst;
    dst.controller_type = yaw ? Cfg::ControllerType::YawController : Cfg::ControllerType::PitchController;
    dst.generic_state_use_to_names[static_cast<size_t>(yaw ? Cfg::GenericControllerStateUse::GimbalYaw : Cfg::GenericControllerStateUse::GimbalPitch)] = yaw ? Cfg::StateName::GimbalYaw : Cfg::StateName::GimbalPitch;
    dst.generic_motor_use_to_names[static_cast<size_t>(yaw ? Cfg::GenericControllerMotorUse::Yaw1 : Cfg::GenericControllerMotorUse::PitchLeft)] = yaw ? Cfg::MotorName::Yaw1 : Cfg::MotorName::Pitch1;
    dst.generic_motor_use_to_names[static_cast<size_t>(yaw ? Cfg::GenericControllerMotorUse::Yaw2 : Cfg::GenericControllerMotorUse::PitchRight)] = yaw ? Cfg::MotorName::Yaw2 : Cfg::MotorName::Pitch2;
    dst.type_to_sub_controller[0].sub_controller_type = Cfg::SubControllerType::FullStatePositionController;
    dst.type_to_sub_controller[0].gains = {src.position_gains[0], src.position_gains[1], src.position_gains[2], src.position_gains[3], 0, 0};
    dst.type_to_sub_controller[1].sub_controller_type = Cfg::SubControllerType::FullStateVelocityController;
    dst.type_to_sub_controller[1].gains = {src.velocity_gains[0], src.velocity_gains[1], src.velocity_gains[2], src.velocity_gains[3], 0, 0};
    dst.gear_ratios.motor1_direction = src.motor_directions[0]; dst.gear_ratios.motor2_direction = src.motor_directions[1];
    dst.gear_ratios.accel_to_normalized_torque = src.accel_to_normalized_torque;
    return dst;
}
Cfg::Estimator estimator_from(const FirmwareSimEstimatorConfig& src) {
    Cfg::Estimator dst; dst.estimator_type = Cfg::EstimatorType::GimbalAndChassis;
    for (size_t i = 0; i < 5; ++i) dst.generic_state_uses_to_names[i] = state_names[i];
    for (size_t i = 0; i < 4; ++i) dst.generic_motor_uses_to_names[i] = static_cast<Cfg::MotorName>(i + 1);
    dst.generic_sensor_uses_to_names[static_cast<size_t>(Cfg::GenericSensorUse::YawBuffEncoder)] = Cfg::SensorName::YawBuffEncoder;
    dst.generic_sensor_uses_to_names[static_cast<size_t>(Cfg::GenericSensorUse::PitchBuffEncoder)] = Cfg::SensorName::PitchBuffEncoder;
    dst.generic_sensor_uses_to_names[static_cast<size_t>(Cfg::GenericSensorUse::YawIcmImu)] = Cfg::SensorName::YawIcmImu;
    auto& info = dst.sensor_info;
    info.yaw_encoder_offset = src.yaw_encoder_offset; info.pitch_encoder_offset = src.pitch_encoder_offset;
    info.yaw_encoder_direction = src.yaw_encoder_direction; info.pitch_encoder_direction = src.pitch_encoder_direction;
    info.yaw_start_angle = src.yaw_start_angle; info.pitch_start_angle = src.pitch_start_angle; info.roll_start_angle = src.roll_start_angle;
    info.pitch_angle_at_imu_calibration = src.pitch_angle_at_imu_calibration;
    std::copy_n(src.yaw_axis_vector, 3, info.yaw_axis_vector); std::copy_n(src.pitch_axis_vector, 3, info.pitch_axis_vector);
    info.has_pitch_imu = src.has_pitch_imu;
    info.chassis_x_to_motor_rad = src.chassis_x_to_motor_rad; info.chassis_y_to_motor_rad = src.chassis_y_to_motor_rad; info.chassis_rad_to_motor_rad = src.chassis_rad_to_motor_rad;
    return dst;
}
bool valid_config(const FirmwareSimConfig& c) {
    for (const auto& s : c.states) {
        if (!finite_values(s.reference_limits, 6) || !finite_values(s.physical_limits, 6) || !std::isfinite(s.max_controller_error) || s.max_controller_error < 0 || s.governor_type > 2 || s.is_wrapping > 1) return false;
        for (size_t j = 0; j < 6; j += 2) if (s.reference_limits[j] > s.reference_limits[j+1] || s.physical_limits[j] > s.physical_limits[j+1]) return false;
        if (s.is_wrapping && s.reference_limits[0] >= s.reference_limits[1]) return false;
        if (s.governor_type != 2 && !(s.reference_limits[4] < 0 && s.reference_limits[5] > 0)) return false;
    }
    for (const auto* ctrl : {&c.yaw, &c.pitch}) {
        if (!finite_values(ctrl->position_gains, 4) || !finite_values(ctrl->velocity_gains, 4) || !std::isfinite(ctrl->accel_to_normalized_torque)) return false;
        for (auto d : ctrl->motor_directions) if (d != -1 && d != 1) return false;
    }
    const auto& e = c.estimator;
    const float scalar_values[] = {e.yaw_encoder_offset, e.pitch_encoder_offset, e.yaw_encoder_direction, e.pitch_encoder_direction, e.yaw_start_angle, e.pitch_start_angle, e.roll_start_angle, e.pitch_angle_at_imu_calibration, e.chassis_x_to_motor_rad, e.chassis_y_to_motor_rad, e.chassis_rad_to_motor_rad};
    if (!finite_values(scalar_values, 11) || !finite_values(e.yaw_axis_vector, 3) || !finite_values(e.pitch_axis_vector, 3) || e.has_pitch_imu > 1) return false;
    if (std::abs(e.yaw_encoder_direction) != 1 || std::abs(e.pitch_encoder_direction) != 1 || e.chassis_x_to_motor_rad == 0 || e.chassis_y_to_motor_rad == 0 || e.chassis_rad_to_motor_rad == 0) return false;
    for (size_t i : {size_t(0), size_t(1), size_t(4), size_t(5), size_t(6), size_t(7)}) if (std::abs(scalar_values[i]) > 1000.0f) return false;
    const double yn = std::hypot(e.yaw_axis_vector[0], e.yaw_axis_vector[1], e.yaw_axis_vector[2]);
    const double pn = std::hypot(e.pitch_axis_vector[0], e.pitch_axis_vector[1], e.pitch_axis_vector[2]);
    if (!(yn > 0 && pn > 0 && yn < std::sqrt(std::numeric_limits<float>::max()) && pn < std::sqrt(std::numeric_limits<float>::max()))) return false;
    return true;
}
}

struct FirmwareSim {
    std::vector<Cfg::State> states;
    Cfg::Controller yaw_config, pitch_config;
    Cfg::Estimator estimator_config;
    CANManager can;
    SensorManager sensors;
    std::vector<Cfg::MotorName> available_motors;
    RobotStateArray estimate, previous, reference, target;
    Governor governor;
    GimbalAndChassisEstimator estimator;
    YawController yaw;
    PitchController pitch;
    uint64_t time_us = 0;
    bool halted = false;
    bool first_loop = true;
    char fault[512] = {};
    explicit FirmwareSim(const FirmwareSimConfig& c) : states(states_from(c)), yaw_config(controller_from(c.yaw, true)), pitch_config(controller_from(c.pitch, false)), estimator_config(estimator_from(c.estimator)), available_motors(std::begin(motor_names), std::end(motor_names)), estimate(states), previous(states), reference(states), target(states), governor(states), estimator(estimator_config, sensors, can, {std::begin(state_names), std::end(state_names)}), yaw(yaw_config, can, available_motors), pitch(pitch_config, can, available_motors) {
        estimate[Cfg::StateName::GimbalYaw].set_position_no_bound(c.estimator.yaw_start_angle);
        estimate[Cfg::StateName::GimbalPitch].set_position_no_bound(c.estimator.pitch_start_angle);
        previous = estimate; target = estimate; reference = estimate;
        governor.set_reference_array(estimate);
        governor.step_reference_array(target); // production first-cycle dt=0 consumed without PID/estimator division by zero
    }
    void initialize_estimate(const float initial[6], float fixed_heading) {
        for (size_t a = 0; a < 2; ++a) {
            auto& s = estimate[state_names[a+3]];
            s.set_position_no_bound(initial[a*3]); s.set_velocity_no_bound(initial[a*3+1]); s.set_acceleration_no_bound(initial[a*3+2]);
        }
        estimator.yaw_angle = initial[0]; estimator.pitch_angle = initial[3];
        estimator.roll_angle = initial[2];
        estimator.current_yaw_velocity = initial[1]; estimator.current_pitch_velocity = initial[4];
        estimate[Cfg::StateName::ChassisHeading].set_position_no_bound(fixed_heading);
        estimator.chassis_angle = fixed_heading;
        estimator.initial_chassis_angle = fixed_heading;
        estimator.prev_global_chassis_angle = fixed_heading;
        estimator.count1 = 1; // Do not replace known initial global attitude with joint angle.
        previous = estimate; target = estimate; reference = estimate;
    }
    void export_output(FirmwareSimOutput& out) {
        out = {}; out.time_us = time_us; out.safety_latched = halted;
        for (size_t axis = 0; axis < 2; ++axis) {
            const auto name = state_names[axis+3]; const auto e = estimate[name].get_raw(); const auto r = reference[name].get_raw();
            out.estimate[axis*3] = e.position; out.estimate[axis*3+1] = e.velocity; out.estimate[axis*3+2] = e.acceleration;
            out.reference[axis*3] = r.position; out.reference[axis*3+1] = r.velocity; out.reference[axis*3+2] = r.acceleration;
        }
        for (size_t i = 0; i < 4; ++i) out.motors[i] = can.get_motor_by_name(motor_names[i])->torque;
    }
    void halt(const char* message) { halted = true; can.zero(); std::snprintf(fault, sizeof(fault), "%s", message); }
};

extern "C" FirmwareSim* firmware_sim_create(const FirmwareSimConfig* config, char* err, uint32_t capacity) {
    error_text(err, capacity, "");
    if (!config || !valid_config(*config)) { error_text(err, capacity, "Invalid firmware simulation configuration (limits, enums, directions or calibration)"); return nullptr; }
    firmware_sim_host::time_us = 0; safety_state.active = false;
    try { return new FirmwareSim(*config); }
    catch (const firmware_sim_host::FatalSafety& f) { error_text(err, capacity, f.message); }
    catch (const std::exception& e) { error_text(err, capacity, e.what()); }
    catch (...) { error_text(err, capacity, "Native simulation initialization failed"); }
    return nullptr;
}
extern "C" int32_t firmware_sim_initialize(FirmwareSim* sim, const float estimate[6], float fixed_heading, char* err, uint32_t capacity) {
    error_text(err, capacity, "");
    if (!sim || sim->time_us != 0 || !estimate || !finite_values(estimate, 6) || !std::isfinite(fixed_heading)) {
        error_text(err, capacity, "Initial estimate must be finite and precede the first cycle"); return -1;
    }
    sim->initialize_estimate(estimate, fixed_heading);
    return 0;
}
extern "C" void firmware_sim_destroy(FirmwareSim* sim) { delete sim; }
extern "C" int32_t firmware_sim_step(FirmwareSim* sim, const FirmwareSimInput* input, FirmwareSimOutput* output, char* err, uint32_t capacity) {
    error_text(err, capacity, ""); if (output) *output = {};
    if (!sim || !input || !output) { error_text(err, capacity, "Null simulation input/output"); return -1; }
    sim->export_output(*output);
    if (sim->halted) { error_text(err, capacity, sim->fault); return 1; }
    if (sim->time_us > UINT64_MAX-1000 || input->time_us != sim->time_us+1000 || input->armed > 1 || input->previous_armed > 1 || input->mode_changed > 1) { error_text(err, capacity, "Expected exactly one 1000us firmware cycle and boolean status flags"); return -1; }
    sim->time_us = input->time_us; firmware_sim_host::time_us = sim->time_us; safety_state.active = !input->previous_armed;
    try {
        if (!finite_values(input->target, 6) || !finite_values(input->sensors, 5)) safety::safety_procedure("Nonfinite simulation command or sensor sample");
        // Keep periodic wrapping bounded even for malicious but finite ABI samples.
        for (size_t i = 0; i < 2; ++i) if (std::abs(input->sensors[i]) > 1000.0f) safety::safety_procedure("Encoder input exceeds raw radian sensor range");
        for (size_t i = 2; i < 5; ++i) if (std::abs(input->sensors[i]) > 10000.0f) safety::safety_procedure("Gyro input exceeds simulated rad/s range");
        sim->sensors.yaw->angle = input->sensors[0]; sim->sensors.pitch->angle = input->sensors[1];
        std::copy_n(input->sensors+2, 3, sim->sensors.imu->gyro);
        for (size_t a = 0; a < 2; ++a) {
            auto& t = sim->target[state_names[a+3]];
            // wrap() is constant time; clamping setters retain the production reference envelope.
            t.set_position(input->target[a*3]); t.set_velocity(input->target[a*3+1]); t.set_acceleration(input->target[a*3+2]);
        }
        sim->previous = sim->estimate;
        sim->estimator.step_states(sim->estimate, sim->previous, 0);
        sim->estimator.validate(sim->estimate);
        // HelloRobot always steps controls, then evaluates current safety and zeroes
        // actuation. Disarming does not reset turret integrators or hold its governor.
        if (sim->first_loop || input->mode_changed) {
            sim->governor.set_reference_array(sim->estimate);
            sim->first_loop = false;
        }
        sim->reference = sim->governor.step_reference_array(sim->target);
        sim->yaw.validate(sim->reference, sim->estimate); sim->pitch.validate(sim->reference, sim->estimate);
        sim->yaw.step(sim->reference, sim->estimate, sim->target); sim->pitch.step(sim->reference, sim->estimate, sim->target);
        safety_state.active = !input->armed;
        if (!input->armed) sim->can.zero();
        // Production nonfinite checks guard states; never allow invalid sink values into the plant.
        for (auto n : motor_names) if (!std::isfinite(sim->can.get_motor_by_name(n)->torque)) safety::safety_procedure("Nonfinite firmware motor torque");
    } catch (const firmware_sim_host::FatalSafety& f) { sim->halt(f.message); }
      catch (const std::exception& e) { sim->halt(e.what()); }
      catch (...) { sim->halt("Unhandled native simulation fault"); }
    sim->export_output(*output); if (sim->halted) { error_text(err, capacity, sim->fault); return 1; }
    return 0;
}
extern "C" void firmware_sim_layout(uint32_t sizes[6]) {
    if (!sizes) return;
    const uint32_t values[6] = {sizeof(FirmwareSimStateConfig), sizeof(FirmwareSimControllerConfig), sizeof(FirmwareSimEstimatorConfig), sizeof(FirmwareSimConfig), sizeof(FirmwareSimInput), sizeof(FirmwareSimOutput)};
    std::copy_n(values, 6, sizes);
}
