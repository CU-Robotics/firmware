#include "control.hpp"

float powerlimit_ratio (){//return powerlimit_ratio (Because we might use the number at the same time, better not call it in each step)   
    float power_buffer = ref.data.power_heat.buffer_energy;
    return constrain(power_buffer / 20, 0.0, 1.0);
} 
void Control::init_controller(uint8_t can_id, uint8_t motor_id, int controller_type) {
    int index = ((can_id-1)*NUM_MOTORS_PER_BUS) + (motor_id-1);

    switch (controller_type) {
        case 0:
            controllers[index] = new NullController();
            break;
        default:
            controllers[index] = new NullController();
            break;
    }
}

void Control::step(float reference[STATE_LEN][3], float estimate[STATE_LEN][3], float kinematics[NUM_MOTORS][STATE_LEN]) {
    // Iterate through motors
    for (int m = 0; m < NUM_MOTORS; m++) {
        Controller controller = controllers[m];
        float output = 0;
        for (int j = 0; j < STATE_LEN; j++) {
            if (kinematics[m][j] == 0) continue;
            output += controller.step(reference[j], estimate[j]) * kinematics[m][j];
        }
    }
}