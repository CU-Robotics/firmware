#include <Arduino.h>

#include "git_info.h"

#include "utils/profiler.hpp"
#include "sensors/d200.hpp"
#include "sensors/StereoCamTrigger.hpp"
#include "controls/estimator_manager.hpp"
#include "controls/controller_manager.hpp"
#include "filters/IMU_filter.hpp"

#include <TeensyDebug.h>
#include "sensors/LEDBoard.hpp"

// Loop constants
#define LOOP_FREQ 1000
#define HEARTBEAT_FREQ 2

// Declare global objects
Timer loop_timer;
Timer stall_timer;
ICM20649 imu;
IMU_filter imu_filter;
// DONT put anything else in this function. It is not a setup function
void print_logo() {
    if (Serial) {
        Serial.println("TEENSY SERIAL START\n\n");
        Serial.print("\033[1;33m");
        Serial.println("                  .:^!?!^.                        ");
        Serial.println("           .:~!?JYYYJ?7?Y5Y7!!.                   ");
        Serial.println("         :?5YJ?!~:.      ^777YP?.                 ");
        Serial.println("         5G~                  ~YP?:               ");
        Serial.println("         7P5555Y:               ^YP?:....         ");
        Serial.println("        ~55J7~^.   ..    .        ^JYYYYYYYYYJJ!. ");
        Serial.println("        YG^     !Y5555J:^PJ    Y5:      ...::^5G^ ");
        Serial.println("       :GY    .YG?^..^~ ~GY    5G^ ^!~~^^^!!~7G?  ");
        Serial.println(" .!JYYY5G!    7BJ       ~GY    5G^ ~??JJJY555GP!  ");
        Serial.println("^55!^:.^~.    ^PP~   .: ^GP:  ^PP:           :7PY.");
        Serial.println("YG^            :JP5YY55: ~YP55PY^              ~GJ");
        Serial.println("?G~      .?7~:   .^~~^.    .^:.                :G5");
        Serial.println(".5P^     7BYJ5YJ7^.                          .~5P^");
        Serial.println(" .JPJ!~!JP?  .:~?PP^            .:.    .^!JYY5Y!. ");
        Serial.println("   :!???!:       5P.         .!Y5YYYJ?Y5Y?!^:.    ");
        Serial.println("                 7G7        7GY!. .:~!^.          ");
        Serial.println("                  JG!      :G5                    ");
        Serial.println("                   7PY!^^~?PY:                    ");
        Serial.println("                    .!JJJJ?^                      ");
        Serial.print("\033[0m");
        Serial.println("\n\033[1;92mFW Ver. 2.1.0");
        Serial.printf("\nLast Built: %s at %s", __DATE__, __TIME__);
        Serial.printf("\nGit Hash: %s", GIT_COMMIT_HASH);
        Serial.printf("\nGit Branch: %s", GIT_BRANCH);
        Serial.printf("\nCommit Message: %s", GIT_COMMIT_MSG);
        Serial.printf("\nRandom Num: %x", ARM_DWT_CYCCNT);
        Serial.println("\033[0m\n");
    }
}
//////////////Example of using IMU_filter////////////////
// Please updata the IMU_filter.cpp and IMU_filter.hpp in the filters folder
// And IMUSensor.cpp and IMUSensor.hpp in the sensors folder
// And All the Sensor's hpp and cpp file in the sensors folder
// float accel_X = 0; //acceleration raw value
// float accel_Y = 0;
// float accel_Z = 0;
// float gyro_X = 0; //raw gyroscope value (rad/s) x(along roll) y(along pitch) z(up-down)
// float gyro_Y = 0;
// float gyro_Z = 0; 
// float temperature = 0; //(c)
// float pitch = 0; //Angle (rad)
// float roll = 0;
// float yaw = 0;
// float accel_world_X = 0; //Acceleration in world frame (m/s)
// float accel_world_Y = 0;
// float accel_world_Z = 0;
// float gyro_pitch = 0; //Filtered angular velocity (rad/s)
// float gyro_roll = 0;
// float gyro_yaw = 0;
// INIT section
    // SPI.begin(); // Start SPI for IMU
    // imu.init(imu.SPI); // Initialize IMU 
    // imu.set_gyro_range(4000); // Set gyro range to 4000 dps
    // imu.calibration_all(); // Calibrate IMU
    // imu_filter.init_EKF_6axis(imu.get_data()); // Initialize EKF filter
// LOOP section
    // imu.read();
    // imu.fix_raw_data();
    // imu_filter.step_EKF_6axis(imu.get_data());
    // IMU_data* filtered_data = imu_filter.get_filter_data();
// Good luck!!! And feel free to ask me (Github: OAOjim / Name: YiChun(Jim) Liao) any question
////////////////Example of using IMU_filter////////////////

// Master loop
int main() {
    long long loopc = 0; // Loop counter for heartbeat
    Serial.begin(115200); // the serial monitor is actually always active (for debug use Serial.println & tycmd)
    debug.begin(SerialUSB1);

    print_logo();
    
    SPI.begin(); // Start SPI for IMU
    imu.init(imu.SPI); // Initialize IMU 
    imu.set_gyro_range(4000); // Set gyro range to 4000 dps
    imu.calibration_all(); // Calibrate IMU
    imu_filter.init_EKF_6axis(imu.get_data()); // Initialize EKF filter
    // Main loop
    bool skip = true;
    while (true) {
        imu.read();
        imu.fix_raw_data();

        imu_filter.step_EKF_6axis(imu.get_data());
        IMU_data* filtered_data = imu_filter.get_filter_data();
        
        
        // // LED heartbeat -- linked to loop count to reveal slowdowns and freezes.
        // loopc % (int)(1E3 / float(HEARTBEAT_FREQ)) < (int)(1E3 / float(5 * HEARTBEAT_FREQ)) ? digitalWrite(13, HIGH) : digitalWrite(13, LOW);
        // loopc++;

        // Keep the loop running at the desired rate
        loop_timer.delay_micros((int)(1E6 / (float)(LOOP_FREQ)));
        float dt = stall_timer.delta();
        if (dt > 0.002) Serial.printf("Slow loop with dt: %f\n", dt);
    }
    
    return 0;
}


