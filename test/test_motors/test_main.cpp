#include <Arduino.h>
#include <unity.h>

#include <cstring>
#include <vector>

#include "sensors/can/SDC104.hpp"
#include "sensors/can/SteadywinGIM.hpp"

namespace {
constexpr uint32_t GIM_ID = 3;
constexpr uint32_t SDC104_NODE = 2;

constexpr uint8_t GIM_CMD_START_MOTOR = 0x91;
constexpr uint8_t GIM_CMD_TORQUE_CONTROL = 0x93;

constexpr uint8_t SDC104_CMD_HEARTBEAT = 0x01;
constexpr uint8_t SDC104_CMD_SET_AXIS_STATE = 0x07;
constexpr uint8_t SDC104_CMD_GET_ENCODER_ESTIMATES = 0x09;
constexpr uint8_t SDC104_CMD_SET_CONTROLLER_MODE = 0x0B;
constexpr uint8_t SDC104_CMD_SET_INPUT_TORQUE = 0x0E;
constexpr uint8_t SDC104_CMD_CLEAR_ERRORS = 0x18;
constexpr uint8_t SDC104_AXIS_IDLE = 1;
constexpr uint8_t SDC104_AXIS_CLOSED_LOOP = 8;

Cfg::Motor make_config(Cfg::MotorControllerType controller, Cfg::MotorType type, uint32_t id) {
    Cfg::Motor config;
    config.motor_controller_type = controller;
    config.physical_bus = 0;
    config.physical_id = id;
    config.motor_type = type;
    config.motor_name = Cfg::MotorName::Feeder;
    return config;
}

float float_at(const uint8_t* bytes) {
    float value;
    std::memcpy(&value, bytes, sizeof(value));
    return value;
}

// GIM helpers

CAN_message_t gim_torque_reply(uint16_t position_raw, uint8_t res = 0x00) {
    CAN_message_t msg;
    msg.id = GIM_ID;
    msg.bus = 1; // 1-indexed
    msg.buf[0] = GIM_CMD_TORQUE_CONTROL;
    msg.buf[1] = res;
    msg.buf[3] = position_raw & 0xFF;
    msg.buf[4] = position_raw >> 8;
    // zero speed and torque are mid-range
    msg.buf[5] = 0x80;
    msg.buf[6] = 0x08;
    msg.buf[7] = 0x00;
    return msg;
}

CAN_message_t gim_output(const SteadywinGIM& motor) {
    CAN_message_t msg;
    motor.write(msg);
    return msg;
}

// a GIM that has been started and has replied once
SteadywinGIM healthy_gim() {
    SteadywinGIM motor(make_config(Cfg::MotorControllerType::GIM, Cfg::MotorType::GIM4310, GIM_ID));
    motor.init();
    gim_output(motor); // send the start command
    CAN_message_t reply = gim_torque_reply(32768);
    motor.read(reply);
    return motor;
}

// SDC104 helpers

CAN_message_t sdc104_reply(uint8_t cmd) {
    CAN_message_t msg;
    msg.id = (SDC104_NODE << 5) | cmd;
    msg.bus = 1;
    return msg;
}

CAN_message_t sdc104_heartbeat(uint8_t axis_state, uint32_t axis_error = 0) {
    CAN_message_t msg = sdc104_reply(SDC104_CMD_HEARTBEAT);
    std::memcpy(&msg.buf[0], &axis_error, sizeof(axis_error));
    msg.buf[4] = axis_state;
    return msg;
}

CAN_message_t sdc104_encoder_estimates(float position_rev, float velocity_rev_s) {
    CAN_message_t msg = sdc104_reply(SDC104_CMD_GET_ENCODER_ESTIMATES);
    std::memcpy(&msg.buf[0], &position_rev, sizeof(float));
    std::memcpy(&msg.buf[4], &velocity_rev_s, sizeof(float));
    return msg;
}

// collect a full write cycle: any queued one-shots plus the 4 alternating frames
std::vector<CAN_message_t> sdc104_frames(const SDC104& motor) {
    std::vector<CAN_message_t> frames;
    for (int i = 0; i < 8; i++) {
        CAN_message_t msg;
        motor.write(msg);
        frames.push_back(msg);
    }
    return frames;
}

bool has_cmd(const std::vector<CAN_message_t>& frames, uint8_t cmd) {
    for (const CAN_message_t& msg : frames) {
        if ((msg.id & 0x1F) == cmd) return true;
    }
    return false;
}

// the input torque the frames command, which should be the same in every command frame
float commanded_input_torque(const std::vector<CAN_message_t>& frames) {
    for (const CAN_message_t& msg : frames) {
        if ((msg.id & 0x1F) == SDC104_CMD_SET_INPUT_TORQUE) return float_at(msg.buf);
    }
    TEST_FAIL_MESSAGE("no set input torque frame");
    return 0.0f;
}

// an SDC104 reporting fresh encoder estimates, closed loop and no fault, already in torque mode
SDC104 armed_sdc104() {
    SDC104 motor(make_config(Cfg::MotorControllerType::SDC104, Cfg::MotorType::GIM6010, SDC104_NODE));
    motor.init();
    sdc104_frames(motor); // send the idle request
    CAN_message_t estimates = sdc104_encoder_estimates(0.0f, 0.0f);
    motor.read(estimates);
    // first call switches the control mode
    motor.write_motor_torque(0.0f);
    sdc104_frames(motor);
    CAN_message_t heartbeat = sdc104_heartbeat(SDC104_AXIS_CLOSED_LOOP);
    motor.read(heartbeat);
    return motor;
}
} // namespace

// GIM

void test_gim_position_unwraps_across_the_boundary(void) {
    host_millis = 1000;
    SteadywinGIM motor = healthy_gim();

    CAN_message_t near_top = gim_torque_reply(65500);
    motor.read(near_top);
    float before = motor.get_position_rad();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.49f, before);

    // forward past the wrap point: the raw count rolls over to near zero
    CAN_message_t past_top = gim_torque_reply(30);
    motor.read(past_top);
    float after = motor.get_position_rad();
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 66.0f * 25.0f / 65535.0f, after - before);

    // and back again
    motor.read(near_top);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, before, motor.get_position_rad());
}

void test_gim_healthy_after_a_status_reply(void) {
    host_millis = 1000;
    SteadywinGIM motor = healthy_gim();
    TEST_ASSERT_TRUE(motor.is_healthy());

    motor.write_motor_torque(0.5f);
    CAN_message_t out = gim_output(motor);
    TEST_ASSERT_EQUAL_HEX8(GIM_CMD_TORQUE_CONTROL, out.buf[0]);
    TEST_ASSERT_TRUE(float_at(&out.buf[1]) > 0.0f);
}

void test_gim_stale_zeroes_torque_and_restarts(void) {
    host_millis = 1000;
    SteadywinGIM motor = healthy_gim();

    host_millis += 300; // no reply for longer than the stale timeout and the restart retry
    TEST_ASSERT_FALSE(motor.is_healthy());

    // the first command while unhealthy resends start instead
    motor.write_motor_torque(0.5f);
    TEST_ASSERT_EQUAL_HEX8(GIM_CMD_START_MOTOR, gim_output(motor).buf[0]);

    // after that, zero torque until the motor replies
    motor.write_motor_torque(0.5f);
    CAN_message_t out = gim_output(motor);
    TEST_ASSERT_EQUAL_HEX8(GIM_CMD_TORQUE_CONTROL, out.buf[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, float_at(&out.buf[1]));

    CAN_message_t reply = gim_torque_reply(32768);
    motor.read(reply);
    TEST_ASSERT_TRUE(motor.is_healthy());
    motor.write_motor_torque(0.5f);
    out = gim_output(motor);
    TEST_ASSERT_TRUE(float_at(&out.buf[1]) > 0.0f);
}

void test_gim_start_command_is_not_overwritten_before_it_is_sent(void) {
    host_millis = 1000;
    SteadywinGIM motor(make_config(Cfg::MotorControllerType::GIM, Cfg::MotorType::GIM4310, GIM_ID));
    motor.init();
    // controller and zero_motor both run before the first write
    motor.write_motor_torque(0.5f);
    motor.zero_motor();
    TEST_ASSERT_EQUAL_HEX8(GIM_CMD_START_MOTOR, gim_output(motor).buf[0]);
}

void test_gim_rejected_command_is_unhealthy(void) {
    host_millis = 1000;
    SteadywinGIM motor = healthy_gim();

    CAN_message_t rejected = gim_torque_reply(32768, 0x01);
    motor.read(rejected);
    TEST_ASSERT_FALSE(motor.is_healthy());
}

// SDC104

void test_sdc104_not_armed_without_encoder_estimates(void) {
    host_millis = 1000;
    SDC104 motor(make_config(Cfg::MotorControllerType::SDC104, Cfg::MotorType::GIM6010, SDC104_NODE));
    motor.init();
    sdc104_frames(motor);

    motor.write_motor_torque(0.5f);
    std::vector<CAN_message_t> frames = sdc104_frames(motor);
    TEST_ASSERT_FALSE(has_cmd(frames, SDC104_CMD_SET_CONTROLLER_MODE));
    TEST_ASSERT_FALSE(has_cmd(frames, SDC104_CMD_SET_AXIS_STATE));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, commanded_input_torque(frames));
}

void test_sdc104_armed_motor_takes_torque(void) {
    host_millis = 1000;
    SDC104 motor = armed_sdc104();
    TEST_ASSERT_TRUE(motor.is_healthy());

    motor.write_motor_torque(0.5f);
    TEST_ASSERT_TRUE(commanded_input_torque(sdc104_frames(motor)) > 0.0f);
}

void test_sdc104_stale_sends_zero(void) {
    host_millis = 1000;
    SDC104 motor = armed_sdc104();
    motor.write_motor_torque(0.5f);

    host_millis += 200;
    TEST_ASSERT_FALSE(motor.is_healthy());
    motor.write_motor_torque(0.5f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, commanded_input_torque(sdc104_frames(motor)));
}

void test_sdc104_zero_motor_idles_and_holds_off_rearming(void) {
    host_millis = 1000;
    SDC104 motor = armed_sdc104();
    motor.write_motor_torque(0.5f);

    motor.zero_motor();
    std::vector<CAN_message_t> frames = sdc104_frames(motor);
    TEST_ASSERT_EQUAL_HEX32((SDC104_NODE << 5) | SDC104_CMD_SET_AXIS_STATE, frames[0].id);
    TEST_ASSERT_EQUAL_UINT8(SDC104_AXIS_IDLE, frames[0].buf[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, commanded_input_torque(frames));

    CAN_message_t idle = sdc104_heartbeat(SDC104_AXIS_IDLE);
    motor.read(idle);
    CAN_message_t estimates = sdc104_encoder_estimates(0.0f, 0.0f);

    // a controller write in the same disarmed loop doesn't re-arm it
    host_millis += 1;
    motor.read(estimates);
    motor.write_motor_torque(0.5f);
    frames = sdc104_frames(motor);
    TEST_ASSERT_FALSE(has_cmd(frames, SDC104_CMD_SET_AXIS_STATE));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, commanded_input_torque(frames));

    // once zero_motor stops (armed again), it requests closed loop
    host_millis += 300;
    motor.read(estimates);
    motor.write_motor_torque(0.5f);
    frames = sdc104_frames(motor);
    TEST_ASSERT_TRUE(has_cmd(frames, SDC104_CMD_SET_AXIS_STATE));
    TEST_ASSERT_EQUAL_UINT8(SDC104_AXIS_CLOSED_LOOP, frames[0].buf[0]);
}

void test_sdc104_fault_blocks_arming_until_disarmed(void) {
    host_millis = 1000;
    SDC104 motor = armed_sdc104();

    CAN_message_t faulted = sdc104_heartbeat(SDC104_AXIS_IDLE, 0x00000040);
    motor.read(faulted);
    TEST_ASSERT_FALSE(motor.is_healthy());

    // armed and faulted: no closed loop request, zero command
    host_millis += 300;
    CAN_message_t estimates = sdc104_encoder_estimates(0.0f, 0.0f);
    motor.read(estimates);
    motor.write_motor_torque(0.5f);
    std::vector<CAN_message_t> frames = sdc104_frames(motor);
    TEST_ASSERT_FALSE(has_cmd(frames, SDC104_CMD_SET_AXIS_STATE));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, commanded_input_torque(frames));

    // disarming clears it
    motor.zero_motor();
    frames = sdc104_frames(motor);
    TEST_ASSERT_EQUAL_HEX32((SDC104_NODE << 5) | SDC104_CMD_CLEAR_ERRORS, frames[0].id);
}

void setup() {
    delay(2000);
    UNITY_BEGIN();

    RUN_TEST(test_gim_position_unwraps_across_the_boundary);
    RUN_TEST(test_gim_healthy_after_a_status_reply);
    RUN_TEST(test_gim_stale_zeroes_torque_and_restarts);
    RUN_TEST(test_gim_start_command_is_not_overwritten_before_it_is_sent);
    RUN_TEST(test_gim_rejected_command_is_unhealthy);

    RUN_TEST(test_sdc104_not_armed_without_encoder_estimates);
    RUN_TEST(test_sdc104_armed_motor_takes_torque);
    RUN_TEST(test_sdc104_stale_sends_zero);
    RUN_TEST(test_sdc104_zero_motor_idles_and_holds_off_rearming);
    RUN_TEST(test_sdc104_fault_blocks_arming_until_disarmed);

    UNITY_END();
}

void loop() {}
