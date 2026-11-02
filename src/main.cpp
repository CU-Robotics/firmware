#include "hello_robot.hpp"
#include "sensors/vn100.hpp"

// Master loop
int main() {

    Serial.begin(115200); // the serial monitor is actually always active (for
                          // debug use Serial.println & tycmd)
    debug.begin(SerialUSB1);

    Utils::print_logo();

    // VN100 hardware test: remove the block comment below to enable, then reflash.
    // Runs instead of the normal robot loop; no Hive connection is needed.
    // Serial3: Teensy TX pin 14 -> VN100 RX2, RX pin 15 <- VN100 TX2.
    // Open the USB serial monitor. Rotate the sensor and look for changing YPR
    // and gyro values, advancing timestamps, and LIVE on both packet streams.
    /*
    delay(2000); // Give the USB serial monitor time to connect.
    static vn100 test_imu(Serial3);
    Serial.println("VN100 test: Serial3, sensor port 2, target 921600 baud");
    while (!test_imu.is_initialized()) {
        test_imu.init();
        if (!test_imu.is_initialized()) {
            Serial.println("VN100 init FAILED: check power and RX/TX wiring; retrying...");
            delay(1000);
        }
    }
    Serial.println("VN100 init OK; waiting for IMU and attitude packets...");

    elapsedMillis report_timer;
    uint64_t previous_imu_time = 0;
    uint64_t previous_attitude_time = 0;
    while (true) {
        test_imu.read();
        if (report_timer >= 500) {
            report_timer = 0;
            const auto &data = test_imu.get_data();
            const uint64_t imu_time = data.imu_time.time_startup;
            const uint64_t attitude_time = data.attitude_time.time_startup;
            // Validity alone stays true after reception stops. Check timestamps too.
            Serial.printf("IMU %s t=%llu ns | attitude %s t=%llu ns\n",
                          !data.imu_valid ? "WAITING" :
                              (imu_time != previous_imu_time ? "LIVE" : "STALE"),
                          static_cast<unsigned long long>(imu_time),
                          !data.attitude_valid ? "WAITING" :
                              (attitude_time != previous_attitude_time ? "LIVE" : "STALE"),
                          static_cast<unsigned long long>(attitude_time));
            if (data.imu_valid) {
                Serial.printf("  accel [m/s^2]: %.3f %.3f %.3f | gyro [rad/s]: %.3f %.3f %.3f\n",
                              data.accel.acc[0], data.accel.acc[1], data.accel.acc[2],
                              data.angular_rate.gyro[0], data.angular_rate.gyro[1], data.angular_rate.gyro[2]);
            }
            if (data.attitude_valid) {
                Serial.printf("  YPR [deg]: %.2f %.2f %.2f | temperature: %.2f C\n",
                              data.ypr.yaw, data.ypr.pitch, data.ypr.roll, data.temperature.temperature);
            }
            previous_imu_time = imu_time;
            previous_attitude_time = attitude_time;
        }
        delayMicroseconds(1000); // Poll frequently; only printing is rate limited.
    }
    */

    static HelloRobot robot;
    robot.init();
    robot.run();

    return 0;
}
