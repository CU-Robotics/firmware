struct MotorStateData : Comms::CommsData {
    MotorStateData() : CommsData(Comms::TypeLabel::MotorStateData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(MotorStateData)) { }

    float torque;
    float speed;
    uint16_t position;
    int16_t temperature;

    Cfg::MotorName motor_name;
};