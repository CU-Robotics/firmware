#include "vn100.hpp"

vn100 *vn100::active_reader = nullptr;

vn100::vn100() : vn100(Serial3) {}

vn100::vn100(HardwareSerial &port) {
    // Binding alone does not open the UART during static construction.
    _uart.set_port(&port);

    // Same output layouts as the library's VN100 example implementation.
    imu_message.binary_group = vn::bin::BinaryGroup::TimeGroup | vn::bin::BinaryGroup::ImuGroup;
    imu_message.num_groups = 2;
    imu_message.group_indices[0] = 1;
    imu_message.group_indices[1] = 2;
    imu_message.group_types[0] = static_cast<uint16_t>(vn::bin::TimeTypes::TimeStartup);
    imu_message.group_types[1] = static_cast<uint16_t>(vn::bin::ImuTypes::Accel) |
                                 static_cast<uint16_t>(vn::bin::ImuTypes::AngularRate) |
                                 static_cast<uint16_t>(vn::bin::ImuTypes::SensSat);

    attitude_message.binary_group = vn::bin::BinaryGroup::TimeGroup | vn::bin::BinaryGroup::ImuGroup |
                                    vn::bin::BinaryGroup::AttitudeGroup;
    attitude_message.num_groups = 3;
    attitude_message.group_indices[0] = 1;
    attitude_message.group_indices[1] = 2;
    attitude_message.group_indices[2] = 4;
    attitude_message.group_types[0] = static_cast<uint16_t>(vn::bin::TimeTypes::TimeStartup);
    attitude_message.group_types[1] = static_cast<uint16_t>(vn::bin::ImuTypes::Temperature) |
                                     static_cast<uint16_t>(vn::bin::ImuTypes::Pressure) |
                                     static_cast<uint16_t>(vn::bin::ImuTypes::Mag);
    attitude_message.group_types[2] = static_cast<uint16_t>(vn::bin::AttitudeTypes::Ypr) |
                                     static_cast<uint16_t>(vn::bin::AttitudeTypes::Quaternion) |
                                     static_cast<uint16_t>(vn::bin::AttitudeTypes::LinAccelNed);
}

void vn100::init() {
    if (_initialized) {
        return;
    }

    _data = Data{};
    _uart.set_binary_callback(nullptr);
    if (!_uart.open() || !configure_baudrate() || !configure_outputs()) {
        Serial.println("VN100 wrapper: initialization failed");
        return;
    }

    _uart.set_binary_callback(binary_message_callback);
    _initialized = true;
}

bool vn100::configure_baudrate() {
    constexpr auto target = vn::BaudrateSetting::BAUD_921600;
    constexpr vn::BaudrateSetting candidates[] = {
        target,
        vn::BaudrateSetting::BAUD_115200,
        vn::BaudrateSetting::BAUD_230400,
        vn::BaudrateSetting::BAUD_460800,
        vn::BaudrateSetting::BAUD_128000,
        vn::BaudrateSetting::BAUD_57600,
        vn::BaudrateSetting::BAUD_38400,
        vn::BaudrateSetting::BAUD_19200,
        vn::BaudrateSetting::BAUD_9600,
    };

    for (const auto baud : candidates) {
        _uart.set_baud(baud);
        vn::Model model{};
        if (_uart.read_register(model) != vn::ErrorCode::OK) {
            continue;
        }
        if (baud == target) {
            return true;
        }

        vn::Baudrate setting{};
        setting.baudrate = target;
        if (_uart.write_register(setting) != vn::ErrorCode::OK) {
            return false;
        }
        _uart.set_baud(target);
        return _uart.read_register(model) == vn::ErrorCode::OK;
    }
    return false;
}

bool vn100::configure_outputs() {
    vn::AsyncOutputEnable output{};
    output.enable = false;
    if (_uart.write_register(output) != vn::ErrorCode::OK) {
        return false;
    }

    vn::AsyncDataOutputType ascii_type{};
    ascii_type.ador = vn::Ador::OFF;
    vn::AsyncDataOutputFreq ascii_rate{};
    ascii_rate.adof = vn::Adof::HZ_0;
    if (_uart.write_register(ascii_type) != vn::ErrorCode::OK ||
        _uart.write_register(ascii_rate) != vn::ErrorCode::OK) {
        return false;
    }

    vn::BinaryOutputMessageConfig1 imu_output{};
    imu_output.async_mode = vn::AsyncMode::Serial2; // VN100 serial port 2
    imu_output.rate_divisor = 2;
    imu_output.config = imu_message;
    vn::BinaryOutputMessageConfig2 attitude_output{};
    attitude_output.async_mode = vn::AsyncMode::Serial2;
    attitude_output.rate_divisor = 8;
    attitude_output.config = attitude_message;
    // Disable any third output left behind by a previous device configuration.
    vn::BinaryOutputMessageConfig3 unused_output{};
    if (_uart.write_register(imu_output) != vn::ErrorCode::OK ||
        _uart.write_register(attitude_output) != vn::ErrorCode::OK ||
        _uart.write_register(unused_output) != vn::ErrorCode::OK) {
        return false;
    }

    output.enable = true;
    return _uart.write_register(output) == vn::ErrorCode::OK;
}

void vn100::read() {
    if (!_initialized) {
        return;
    }
    // vn::Serial invokes its callback synchronously inside loop(). All instances
    // are polled on the foreground thread; restore the previous owner afterwards.
    vn100 *previous_reader = active_reader;
    active_reader = this;
    _uart.loop();
    active_reader = previous_reader;
}

void vn100::binary_message_callback(vn::bin::const_bin_buf_ref_t buf,
                                    vn::msg::len_t length, vn::bin::BinaryMessage &msg) {
    if (!active_reader || length != msg.size + 4 + 2 * msg.num_groups) {
        return;
    }

    auto &instance = *active_reader;
    // Commit a group only after every field parses; keep the other group intact.
    Data next = instance._data;
    if (vn::bin::compare_messages(instance.imu_message, msg) == vn::ErrorCode::OK) {
        if (vn::bin::parse_time_startup(buf, msg, next.imu_time) != vn::ErrorCode::OK ||
            vn::bin::parse_accel(buf, msg, next.accel) != vn::ErrorCode::OK ||
            vn::bin::parse_angular_rate(buf, msg, next.angular_rate) != vn::ErrorCode::OK ||
            vn::bin::parse_sens_sat(buf, msg, next.saturation) != vn::ErrorCode::OK) {
            return;
        }
        next.imu_valid = true;
    } else if (vn::bin::compare_messages(instance.attitude_message, msg) == vn::ErrorCode::OK) {
        if (vn::bin::parse_time_startup(buf, msg, next.attitude_time) != vn::ErrorCode::OK ||
            vn::bin::parse_temperature(buf, msg, next.temperature) != vn::ErrorCode::OK ||
            vn::bin::parse_pressure(buf, msg, next.pressure) != vn::ErrorCode::OK ||
            vn::bin::parse_mag(buf, msg, next.mag) != vn::ErrorCode::OK ||
            vn::bin::parse_ypr(buf, msg, next.ypr) != vn::ErrorCode::OK ||
            vn::bin::parse_quaternion(buf, msg, next.quaternion) != vn::ErrorCode::OK ||
            vn::bin::parse_lin_accel_ned(buf, msg, next.lin_accel_ned) != vn::ErrorCode::OK) {
            return;
        }
        next.attitude_valid = true;
    } else {
        return;
    }
    instance._data = next;
}

void vn100::send_to_comms() const {
    // VN100 has no telemetry type in the shared firmware/Hive protocol yet.
}
