#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

#include "sensors/can/can_manager.hpp"

namespace {
constexpr uint32_t kBusId = CAN_1;
constexpr uint32_t kMotorId = 1;

CANManager can_manager;
float commanded_torque = 0.0f;

void configure_motor() {
    float motor_info[CAN_MAX_MOTORS][4] = { 0 };

    motor_info[0][0] = static_cast<float>(static_cast<int>(MotorControllerType::C620));
    motor_info[0][1] = static_cast<float>(kMotorId);
    motor_info[0][2] = static_cast<float>(kBusId);
    motor_info[0][3] = static_cast<float>(static_cast<int>(MotorType::NULL_MOTOR_TYPE));

    can_manager.configure(motor_info);

    // Ensure the motor is idle before any user command
    can_manager.write_motor_torque(kBusId, kMotorId, 0.0f);
    can_manager.write();
}

void maybe_update_torque() {
    if (!Serial.available()) {
        return;
    }

    char buffer[32] = { 0 };
    size_t count = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    if (count == 0) {
        return;
    }

    float value = atof(buffer);
    if (isnan(value) || isinf(value)) {
        Serial.println("Ignoring invalid torque input");
        return;
    }

    if (value > 1.0f) {
        value = 1.0f;
    } else if (value < -1.0f) {
        value = -1.0f;
    }

    commanded_torque = value;
    Serial.printf("Torque command set to %.3f\r\n", commanded_torque);
}
} // namespace

int main() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {
        // wait for the USB serial connection when available
    }

    Serial.println("C620 torque test");
    Serial.println("Enter torque [-1.0, 1.0] followed by newline.");

    can_manager.init();
    configure_motor();

    uint32_t last_print_ms = millis();

    while (true) {
        maybe_update_torque();

        can_manager.write_motor_torque(kBusId, kMotorId, commanded_torque);
        can_manager.write();
        can_manager.read();

        if (millis() - last_print_ms >= 200) {
            const MotorState state = can_manager.get_motor_state(kBusId, kMotorId);
            Serial.printf(
                "Cmd %.3f | measured torque %.3f | speed %.2f rad/s | position %u | temp %d C\r\n",
                commanded_torque,
                state.torque,
                state.speed,
                state.position,
                state.temperature);
            last_print_ms = millis();
        }

        delay(5);
    }

    return 0;
}
