#include "estimator_manager.hpp"

EstimatorManager::EstimatorManager(CANData *data)
{

    pinMode(YAW_BUFF_CS, OUTPUT);
    pinMode(PITCH_BUFF_CS, OUTPUT);
    pinMode(ICM_CS, OUTPUT);

    digitalWrite(YAW_BUFF_CS, HIGH);
    digitalWrite(PITCH_BUFF_CS, HIGH);
    digitalWrite(ICM_CS, HIGH);

    Serial.println("Starting SPI");
    SPI.begin();
    Serial.println("SPI Started");

    buff_sensors[0].init(YAW_BUFF_CS);
    buff_sensors[1].init(PITCH_BUFF_CS);

    icm_sensors[0].init(icm_sensors[0].CommunicationProtocol::SPI);
    icm_sensors[0].set_gyro_range(4000);
    can_data = data;
}

void EstimatorManager::init_estimator(int state_id, int num_states)
{
    switch (state_id)
    {
    case 1: // Gimbal Estimator
        float values_gimbal[10];
        values_gimbal[0] = 0;       // yaw encoder offset
        values_gimbal[1] = 4.09;       // pitch encoder offset
        values_gimbal[2] = 0;       // default yaw starting angle (starting point for imu integration)
        values_gimbal[3] = 1.91986; // default pitch starting angle (starting point for imu integration)
        values_gimbal[4] = 0;       // default roll starting angle (starting point for imu integration)
        values_gimbal[5] = 0;       // default chassis pitch angle
        // Stable gravity vector {x,y,z}
        values_gimbal[6] = -0.05664;  // x
        values_gimbal[7] = 2.057767;  // y
        values_gimbal[8] = 5.544132; // z
        values_gimbal[9] = 1.91986;   // Pitch angle at given gravity vector

        estimators[0] = new GimbalEstimator(values_gimbal, &buff_sensors[0], &buff_sensors[1], &icm_sensors[0], can_data, num_states);
        break;
    case 2:
        estimators[1] = new FlyWheelEstimator(can_data, num_states);
        break;
    case 3:
        estimators[2] = new FeederEstimator(can_data, num_states);
        break;
    case 4:
        estimators[3] = new LocalEstimator(can_data, num_states);
        break;
    default:
        break;
    }
}

void EstimatorManager::step(float macro_outputs[STATE_LEN][3], float micro_outputs[NUM_MOTORS][MICRO_STATE_LEN])
{
    // clear output
    clear_outputs(macro_outputs, micro_outputs);
    for (int i = 0; i < NUM_ESTIMATORS; i++)
    {
        int num_states = estimators[i]->get_num_states();
        float macro_states[STATE_LEN][3] = {0};
        float micro_states[NUM_MOTORS][MICRO_STATE_LEN] = {0};
        
        // memset(micro_states, 0, NUM_MOTORS*MICRO_STATE_LEN * 4);
        if (!estimators[i]->micro_estimator)
        {
            estimators[i]->step_states(macro_states);
            for (int j = 0; j < num_states; j++)
            {
                for (int k = 0; k < 3; k++)
                    macro_outputs[applied_states[i][j]][k] = macro_outputs[applied_states[i][j]][k] + macro_states[j][k];
            }
        }
        else
        {
            estimators[i]->step_states(micro_states);
            // Serial.println(micro_states[9][0]);
            for (int j = 0; j < num_states; j++)
            {
                for (int k = 0; k < MICRO_STATE_LEN; k++){
                    micro_outputs[applied_states[i][j]][k] = micro_outputs[applied_states[i][j]][k] + micro_states[j][k];
                }
            }
        }
    }
}

void EstimatorManager::clear_outputs(float macro_outputs[STATE_LEN][3], float micro_outputs[NUM_MOTORS][MICRO_STATE_LEN])
{
    for (int i = 0; i < STATE_LEN; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            macro_outputs[i][j] = 0;
        }
    }
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        for (int j = 0; j < MICRO_STATE_LEN; j++)
        {
            micro_outputs[i][j] = 0;
        }
    }
}

void EstimatorManager::assign_states(int as[NUM_ESTIMATORS][STATE_LEN])
{
    for (int i = 0; i < NUM_ESTIMATORS; i++)
    {
        for (int j = 0; j < STATE_LEN; j++)
        {
            applied_states[i][j] = as[i][j];
        }
    }
}

void EstimatorManager::read_sensors()
{
    buff_sensors[0].read();
    buff_sensors[1].read();
    icm_sensors[0].read();
}

//Calibrated offsets: 0.255322, -0.017980, 0.000764
//Calibrated offsets: 0.198682, 2.039787, 5.544896
//Calibrated offsets: 3.045174, 0.027272, 0.010071
void EstimatorManager::calibrate_imus()
{
    Serial.println("Calibrating IMU's...");
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;

    float sum_accel_x = 0;
    float sum_accel_y = 0;
    float sum_accel_z = 0;

    for (int i = 0; i < NUM_IMU_CALIBRATION; i++)
    {
        icm_sensors[0].read();
        sum_x += icm_sensors[0].get_gyro_X();
        sum_y += icm_sensors[0].get_gyro_Y();
        sum_z += icm_sensors[0].get_gyro_Z();

        sum_accel_x += icm_sensors[0].get_accel_X();
        sum_accel_y += icm_sensors[0].get_accel_Y();
        sum_accel_z += icm_sensors[0].get_accel_Z();
    }

    Serial.printf("Calibrated offsets: %f, %f, %f", sum_x / NUM_IMU_CALIBRATION, sum_y / NUM_IMU_CALIBRATION, sum_z / NUM_IMU_CALIBRATION);
    Serial.println();
    icm_sensors[0].set_offsets(sum_x / NUM_IMU_CALIBRATION, sum_y / NUM_IMU_CALIBRATION, sum_z / NUM_IMU_CALIBRATION);

    // sum_x = 0;
    // sum_y = 0;
    // sum_z = 0;

    // for(int i = 0; i < NUM_IMU_CALIBRATION; i++){
    //     icm_sensors[0].read();
    //     sum_x += icm_sensors[0].get_gyro_X();
    //     sum_y += icm_sensors[0].get_gyro_Y();
    //     sum_z += icm_sensors[0].get_gyro_Z();
    // }
    // Serial.printf("Calibrated offsets 2: %f, %f, %f", sum_x/NUM_IMU_CALIBRATION, sum_y/NUM_IMU_CALIBRATION, sum_z/NUM_IMU_CALIBRATION);
    // Serial.println();
}


EstimatorManager::~EstimatorManager()
{
    Serial.println("Ending SPI");
    SPI.end();
    Serial.println("SPI Ended");

    for (int i = 0; i < STATE_LEN; i++)
    {
        if (estimators[i] == nullptr)
            continue;
        delete estimators[i];
    }
}