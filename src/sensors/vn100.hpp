#pragma once

#include <Arduino.h>

#include <vn100/protocol/serial.hpp>
#include "sensors/sensor.hpp"

/// @brief Sensor wrapper using the unchanged VN100 serial protocol library.
/// @note Defaults to Teensy Serial3, connected to VN100 serial port 2.
/// Call init() during setup, then read() at 1 kHz from the main loop.
/// All instances must be initialized and polled from the same thread, outside ISRs.
/// Getters retain the last valid sample; check get_data() validity flags before use.
/// See docs/vn100.md for a polling example and integration notes.
/// @see https://docs.google.com/document/d/1nKZURsbD1O32F51RnJQU3J4d7ND6OqpvrFN9zFMxKp4/edit?tab=t.0
class vn100 : public Sensor {
public:
    /// @brief Cached binary output; each packet group updates only after successful parsing.
    struct Data {
        vn::bin::TimeStartup imu_time{}; ///< IMU timestamp in sensor startup nanoseconds.
        vn::bin::Accel accel{}; ///< Body acceleration in m/s^2.
        vn::bin::AngularRate angular_rate{}; ///< Body angular rate in rad/s.
        vn::bin::SensSat saturation{}; ///< Sensor saturation flags.
        vn::bin::TimeStartup attitude_time{}; ///< Attitude timestamp in sensor startup nanoseconds.
        vn::bin::Temperature temperature{}; ///< Temperature in degrees Celsius.
        vn::bin::Pressure pressure{}; ///< Pressure in kPa.
        vn::bin::Mag mag{}; ///< Body magnetic field in Gauss.
        vn::bin::Ypr ypr{}; ///< Yaw, pitch and roll in degrees.
        vn::bin::Quaternion quaternion{}; ///< Quaternion in x, y, z, scalar order.
        vn::bin::LinAccelNed lin_accel_ned{}; ///< NED linear acceleration in m/s^2.
        bool imu_valid{false}; ///< At least one IMU packet has been received.
        bool attitude_valid{false}; ///< At least one attitude packet has been received.
    };

    /// @brief Construct using Serial3; UART setup is deferred until init().
    vn100();
    /// @brief Construct using a selected Teensy UART.
    /// @param port UART connected to VN100 serial port 2; must outlive this sensor.
    explicit vn100(HardwareSerial &port);
    /// @brief UART ownership cannot be copied between wrappers.
    vn100(const vn100 &) = delete;
    /// @brief UART ownership cannot be assigned between wrappers.
    vn100 &operator=(const vn100 &) = delete;

    /// @brief Initialize the driver. Check is_initialized() for success; call again to retry.
    void init() override;
    /// @brief Poll incoming packets if initialized, retaining the last valid readings.
    void read() override;
    /// @brief No-op until a VN100 telemetry type is added to the firmware/Hive protocol.
    void send_to_comms() const override;

    /// @brief Whether sensor configuration succeeded (does not indicate fresh data).
    /// @return True after successful initialization.
    bool is_initialized() const { return _initialized; }
    /// @brief Access both output groups, including validity flags and sensor timestamps.
    /// @return Latest readings; updated by read(). Copy before sharing with another thread/ISR.
    /// @note Validity means a packet was received, not that it is still fresh.
    const Data &get_data() const { return _data; }

    /// @brief Get body X acceleration, including gravity.
    /// @return Acceleration in m/s^2; valid after an IMU packet arrives.
    float get_accel_X() const { return get_data().accel.acc[0]; }
    /// @brief Get body Y acceleration, including gravity.
    /// @return Acceleration in m/s^2; valid after an IMU packet arrives.
    float get_accel_Y() const { return get_data().accel.acc[1]; }
    /// @brief Get body Z acceleration, including gravity.
    /// @return Acceleration in m/s^2; valid after an IMU packet arrives.
    float get_accel_Z() const { return get_data().accel.acc[2]; }

    /// @brief Get angular rate about the body X axis.
    /// @return Angular rate in rad/s; valid after an IMU packet arrives.
    float get_gyro_X() const { return get_data().angular_rate.gyro[0]; }
    /// @brief Get angular rate about the body Y axis.
    /// @return Angular rate in rad/s; valid after an IMU packet arrives.
    float get_gyro_Y() const { return get_data().angular_rate.gyro[1]; }
    /// @brief Get angular rate about the body Z axis.
    /// @return Angular rate in rad/s; valid after an IMU packet arrives.
    float get_gyro_Z() const { return get_data().angular_rate.gyro[2]; }

    /// @brief Get sensor temperature.
    /// @return Temperature in degrees Celsius; valid after an attitude packet arrives.
    float get_temperature() const { return get_data().temperature.temperature; }
    /// @brief Get the sensor's estimated yaw.
    /// @return Yaw in degrees; valid after an attitude packet arrives.
    float get_yaw() const { return get_data().ypr.yaw; }
    /// @brief Get the sensor's estimated pitch.
    /// @return Pitch in degrees; valid after an attitude packet arrives.
    float get_pitch() const { return get_data().ypr.pitch; }
    /// @brief Get the sensor's estimated roll.
    /// @return Roll in degrees; valid after an attitude packet arrives.
    float get_roll() const { return get_data().ypr.roll; }

private:
    /// @brief Probe supported baud rates and switch the device to 921600 baud.
    /// @return True when communication at the target baud rate succeeds.
    bool configure_baudrate();
    /// @brief Configure the two binary output groups using library register commands.
    /// @return True when every configuration command succeeds.
    bool configure_outputs();
    /// @brief Route a synchronous library callback to the instance being polled.
    /// @param buf Complete binary packet, including header and CRC.
    /// @param length Number of bytes in the packet.
    /// @param msg Parsed group layout supplied by the library.
    static void binary_message_callback(vn::bin::const_bin_buf_ref_t buf,
                                       vn::msg::len_t length, vn::bin::BinaryMessage &msg);

    /// @brief Callback owner during read(); the library callback has no context argument.
    static vn100 *active_reader;
    /// @brief UART transport and packet parser from the unmodified library.
    vn::Serial _uart;
    /// @brief Whether the complete configuration handshake succeeded.
    bool _initialized{false};
    /// @brief Latest complete samples from each output group.
    Data _data{};
    /// @brief Expected IMU output layout.
    vn::bin::BinaryMessage imu_message{};
    /// @brief Expected attitude output layout.
    vn::bin::BinaryMessage attitude_message{};
};
