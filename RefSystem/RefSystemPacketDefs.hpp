#ifndef REF_SYSTEM_PACKET_DEFINITIONS_HPP
#define REF_SYSTEM_PACKET_DEFINITIONS_HPP

constexpr uint16_t REF_MAX_FRAME_DATA_SIZE = 128;
constexpr uint16_t REF_MAX_COMMAND_ID = 0x0307;

/// @brief Enum of all frame types corresponding to it's command ID
enum FrameType
{
    GAME_STATUS = 0x0001,
    GAME_RESULT = 0x0002,
    ROBOT_HEALTH = 0x0003,
    EVENT_DATA = 0x0101,
    PROJECTILE_SUPPLY_STATUS = 0x0102,
    REFEREE_WARNING = 0x0104,
    DART_TIME = 0x0105,
    ROBOT_STATUS = 0x0201,
    HEAT_POWER_DATA = 0x0202,
    ROBOT_POSITION = 0x0203,
    BUFF_DATA = 0x0204,
    AIR_SUPPORT_DATA = 0x0206,
    SHOOT_DATA = 0x0207,
    PROJECTILE_ALLOWANCE = 0x0208,
    RFID_DATA = 0x0209,
    DART_CLIENT_COMMAND = 0x020A,
    GROUND_ROBOT_POSITIONS = 0x020B,
    RADAR_MARK_DATA = 0x020C
};

/// @brief Get the name of the FrameType
/// @param type FrameType enum
/// @return string version of the FrameType
static const char* GetFrameName(FrameType type)
{
    switch (type)
    {
    case GAME_STATUS:
        return "GameStatus";
        break;
    case GAME_RESULT:
        return "GameResult";
        break;
    case ROBOT_HEALTH:
        return "RobotHealth";
        break;
    case EVENT_DATA:
        return "EventData";
        break;
    case PROJECTILE_SUPPLY_STATUS:
        return "ProjectileSupplyStatus";
        break;
    case REFEREE_WARNING:
        return "RefereeWarning";
        break;
    case DART_TIME:
        return "DartTime";
        break;
    case HEAT_POWER_DATA:
        return "HeatPowerData";
        break;
    case ROBOT_POSITION:
        return "RobotPosition";
        break;
    case BUFF_DATA:
        return "BuffData";
        break;
    case AIR_SUPPORT_DATA:
        return "AirSupportData";
        break;
    case SHOOT_DATA:
        return "ShootData";
        break;
    case PROJECTILE_ALLOWANCE:
        return "ProjectileAllowance";
        break;
    case RFID_DATA:
        return "RFIDdata";
        break;
    case DART_CLIENT_COMMAND:
        return "DartClientCommand";
        break;
    case GROUND_ROBOT_POSITIONS:
        return "GroundRobotPositions";
        break;
    case RADAR_MARK_DATA:
        return "RadarMarkData";
        break;

    default:
        return "Invalid Frame ID";
        break;
    }
}

/// @brief Header portion of a complete Frame
struct FrameHeader
{
    static const uint8_t size = 5;

    /// @brief Start of frame byte (should always be 0xA5)
    uint8_t SOF = 0;
    /// @brief Length of the data frame portion
    uint16_t data_length = 0;
    /// @brief Sequence number, counts up, wraps around 255
    uint8_t seq = 0;
    /// @brief 8-bit CRC for the Frame header
    uint8_t CRC8 = 0;
};

/// @brief Data portion of a complete Frame
struct FrameData
{
    /// @brief Simple byte array to store the data portion of a Frame
    uint8_t data[REF_MAX_FRAME_DATA_SIZE] = { 0 };

    uint8_t operator[](uint16_t index)
    {
        return data[index];
    }
};

/// @brief Competition status data. Transmitted at 3Hz to all robots
/// @note ID: 0x0001
struct GameStatus
{
    /// @brief Competition Type (1: Uni Championship, 2: Uni Tech Challenge, 3: ICRA AI Challenge, 4: 3v3 Match, 5: Uni Standard Match)
    uint8_t game_type = 0;
    /// @brief Game Progress (0: Pre-comp, 1: Setup, 2: Initialization, 3: 5-seconds, 4: Ongoing, 5: Calculating Results)
    uint8_t game_progress = 0;
    /// @brief Remaining time in the current round (in seconds)
    uint16_t remaining_round_time = 0;
    /// @brief UNIX time, effective after the robot is correctly linked to the Referee System’s NTP server
    uint64_t current_time = 0;

    void InitializeFromRaw(FrameData& data)
    {
        game_type = data[0];
        game_progress = data[1];

        remaining_round_time = (data[3] << 8) | data[2];

        current_time = data[11];
        current_time = (current_time << 8) | data[10];
        current_time = (current_time << 8) | data[9];
        current_time = (current_time << 8) | data[8];
        current_time = (current_time << 8) | data[7];
        current_time = (current_time << 8) | data[6];
        current_time = (current_time << 8) | data[5];
        current_time = (current_time << 8) | data[4];
    }

    void print()
    {
        Serial.printf("GameStatus packet: \n");
        Serial.printf("Game Type: %.2x\n", game_type);
        Serial.printf("Game Progress: %.2x\n", game_progress);
        Serial.printf("Remaining Round Time: %.4x\n", remaining_round_time);
        Serial.printf("Current Time: %u\n", current_time);
    }
};

/// @brief Game result data. Transmitted at the end of the competition
/// @note ID: 0x0002
struct GameResult
{
    /// @brief Winner of the game (0: Draw, 1: Red, 2: Blue)
    uint8_t winner = 0;

    void InitializeFromRaw(FrameData& data)
    {
        winner = data[0];
    }

    void print()
    {
        Serial.printf("Game Result packet: \n");
        Serial.printf("Game Winner: %x\n", winner);
    }
};

/// @brief Global robot health data. Transmitted at 3Hz to all robots
/// @note ID: 0x0003
struct RobotHealth
{
    uint16_t red_hero_HP = 0;
    uint16_t red_engineer_HP = 0;
    uint16_t red_standard_1_HP = 0;
    uint16_t red_standard_2_HP = 0;
    uint16_t red_standard_3_HP = 0;
    uint16_t red_sentry_HP = 0;
    uint16_t red_outpost_HP = 0;
    uint16_t red_base_HP = 0;

    uint16_t blue_hero_HP = 0;
    uint16_t blue_engineer_HP = 0;
    uint16_t blue_standard_1_HP = 0;
    uint16_t blue_standard_2_HP = 0;
    uint16_t blue_standard_3_HP = 0;
    uint16_t blue_sentry_HP = 0;
    uint16_t blue_outpost_HP = 0;
    uint16_t blue_base_HP = 0;

    void InitializeFromRaw(FrameData& data)
    {
        red_hero_HP = (data[1] << 8) | data[0];
        red_engineer_HP = (data[3] << 8) | data[2];
        red_standard_1_HP = (data[5] << 8) | data[4];
        red_standard_2_HP = (data[7] << 8) | data[6];
        red_standard_3_HP = (data[9] << 8) | data[8];
        red_sentry_HP = (data[11] << 8) | data[10];
        red_outpost_HP = (data[13] << 8) | data[12];
        red_base_HP = (data[15] << 8) | data[14];

        blue_hero_HP = (data[17] << 8) | data[16];
        blue_engineer_HP = (data[19] << 8) | data[18];
        blue_standard_1_HP = (data[21] << 8) | data[20];
        blue_standard_2_HP = (data[23] << 8) | data[22];
        blue_standard_3_HP = (data[25] << 8) | data[24];
        blue_sentry_HP = (data[27] << 8) | data[26];
        blue_outpost_HP = (data[29] << 8) | data[28];
        blue_base_HP = (data[31] << 8) | data[30];
    }

    void print()
    {
        Serial.printf("Robot Health packet: \n");
        Serial.printf("Red hero Health: \n", red_hero_HP);
        Serial.printf("Red engineer Health: \n", red_engineer_HP);
        Serial.printf("Red standard_1 Health: \n", red_standard_1_HP);
        Serial.printf("Red standard_2 Health: \n", red_standard_2_HP);
        Serial.printf("Red standard_3 Health: \n", red_standard_3_HP);
        Serial.printf("Red sentry Health: \n", red_sentry_HP);
        Serial.printf("Red outpost Health: \n", red_outpost_HP);
        Serial.printf("Red base Health: \n", red_base_HP);

        Serial.printf("Blue hero Health: \n", blue_hero_HP);
        Serial.printf("Blue engineer Health: \n", blue_engineer_HP);
        Serial.printf("Blue standard_1 Health: \n", blue_standard_1_HP);
        Serial.printf("Blue standard_2 Health: \n", blue_standard_2_HP);
        Serial.printf("Blue standard_3 Health: \n", blue_standard_3_HP);
        Serial.printf("Blue sentry Health: \n", blue_sentry_HP);
        Serial.printf("Blue outpost Health: \n", blue_outpost_HP);
        Serial.printf("Blue base Health: \n", blue_base_HP);
    }
};

/// @brief Site Event Data. Transmitted at 3Hz to all of our robots
/// @note ID: 0x0101
struct EventData
{
    /// @brief Occupation status of the Restoration Zone in front of own Official Projectile Supplier
    uint8_t is_restoration_zone_front_occupied = 0;
    /// @brief Occupation status of the Restoration Zone to the left of own Official Projectile Supplier
    uint8_t is_restoration_zone_left_occupied = 0;
    /// @brief Occupation status of the Restoration Zone to the right of own Official Projectile Supplier
    uint8_t is_restoration_zone_right_occupied = 0;

    /// @brief Occupation status of own Power Rune Activation Point
    uint8_t is_power_rune_occupied = 0;
    /// @brief Activation status of own Small Power Rune
    uint8_t is_small_power_rune_activated = 0;
    /// @brief Activation status of own Large Power Rune
    uint8_t is_large_power_rune_activated = 0;

    /// @brief Occupation status of own Ring-Shaped Elevated Ground
    uint8_t is_ring_elevated_ground_occupied = 0;
    /// @brief Occupation status of own R3 Trapezoid-Shaped Elevated Ground
    uint8_t is_R3_trapezoid_elevated_ground_occupied = 0;
    /// @brief Occupation status of own R4 Trapezoidal Elevated Ground
    uint8_t is_R4_trapezoid_elevated_ground_occupied = 0;

    /// @brief Value of own Base’s Virtual Shield (0-250)
    uint8_t virtual_shield_HP = 0;
    /// @brief HP of own Outpost (0-1500)
    uint16_t outpost_HP = 0;
    /// @brief Whether the Sentry is in own Patrol Zone
    uint8_t is_sentry_in_patrol_zone = 0;

    void InitializeFromRaw(FrameData& data)
    {
        is_restoration_zone_front_occupied = data[0] & 0x80;
        is_restoration_zone_left_occupied = data[0] & 0x40;
        is_restoration_zone_right_occupied = data[0] & 0x20;

        is_power_rune_occupied = data[0] & 0x10;
        is_small_power_rune_activated = data[0] & 0x08;
        is_large_power_rune_activated = data[0] & 0x04;

        is_ring_elevated_ground_occupied = data[0] & 0x02;
        is_R3_trapezoid_elevated_ground_occupied = data[0] & 0x01;
        is_R4_trapezoid_elevated_ground_occupied = data[1] & 0x80;

        virtual_shield_HP = ((data[1] & 0x7f) << 1) | (data[2] & 0x80);
        outpost_HP = ((data[2] & 0x7f) << 4) | (data[3] & 0xf0);
        is_sentry_in_patrol_zone = data[3] & 0x08;
    }

    void print()
    {
        Serial.printf("Event Data packet: \n");
        Serial.printf("Is Restoration Zone Front Occupied: %x\n", is_restoration_zone_front_occupied);
        Serial.printf("Is Restoration Zone Left Occupied: %x\n", is_restoration_zone_left_occupied);
        Serial.printf("Is Restoration Zone Right Occupied: %x\n", is_restoration_zone_right_occupied);

        Serial.printf("Is Power Rune Occupied: %x\n", is_power_rune_occupied);
        Serial.printf("Is Small Power Rune Activated: %x\n", is_small_power_rune_activated);
        Serial.printf("Is Large Power Rune Activated: %x\n", is_large_power_rune_activated);

        Serial.printf("Is Ring Shaped Elevated Ground Occupied: %x\n", is_ring_elevated_ground_occupied);
        Serial.printf("Is R3 Trapizoid Shaped Elevated Ground Occupied: %x\n", is_R3_trapezoid_elevated_ground_occupied);
        Serial.printf("Is R4 Trapizoid Shaped Ground Occupied: %x\n", is_R4_trapezoid_elevated_ground_occupied);

        Serial.printf("Virtual Shield HP: %u\n", virtual_shield_HP);
        Serial.printf("Outpost HP: %u\n", outpost_HP);
        Serial.printf("Is Sentry in Patrol Zone: %x\n", is_sentry_in_patrol_zone);
    }
};

/// @brief Official Projectile Supplier action identifier data. Transmitted only when Supplier releases projectiles. Sent to all of our robots
/// @note ID: 0x0102
struct ProjectileSupplyStatus
{
    /// @brief Official Projectile Supplier Outlet ID. 1 = left, 2 = right
    uint8_t supply_projectile_id = 0;
    /// @brief Reloading robot ID. Blue IDs are ID + 100 (i.e. blue hero is 101, while red is 1)
    uint8_t supply_robot_id = 0;
    /// @brief Status of the projectile outlet (0 = closed, 1 = preparing, 2 = releasing)
    uint8_t supply_projectile_status = 0;
    /// @brief Number of supplied projectiles
    uint8_t supply_projectile_count = 0;

    void InitializeFromRaw(FrameData& data)
    {
        supply_projectile_id = data[0];
        supply_robot_id = data[1];
        supply_projectile_status = data[2];
        supply_projectile_count = data[3];
    }

    void print()
    {
        Serial.printf("Projectile Supplier Status: \n");
        Serial.printf("Supplier ID: %x\n", supply_projectile_id);
        Serial.printf("Robot ID: %x\n", supply_robot_id);
        Serial.printf("Supplier Status: %x\n", supply_projectile_status);
        Serial.printf("Supplied Projectiles: %u\n", supply_projectile_count);
    }
};

/// @brief Referee warning data. Transmitted if any warnings are issued to our team (all of our robots)
/// @note ID: 0x0104
struct RefereeWarning
{
    /// @brief Penalty level (1: Yellow, 2: Red, 3: Forfeiture)
    uint8_t level = 0;
    /// @brief ID of robot being issued a penalty
    uint8_t offending_robot_id = 0;

    void InitializeFromRaw(FrameData& data)
    {
        level = data[0];
        offending_robot_id = data[1];
    }

    void print()
    {
        Serial.printf("Referee Warning packet: \n");
        Serial.printf("Penalty Level: %x\n", level);
        Serial.printf("Offending Robot ID: %x\n", offending_robot_id);
    }
};

/// @brief Dart launching time data. Transmitted at 3Hz to all of our team's robots
/// @note ID: 0x0105
struct DartTime
{
    /// @brief Own team’s remaining time for dart launching (in seconds)
    uint8_t dart_remaining_time = 0;

    void InitializeFromRaw(FrameData& data)
    {
        dart_remaining_time = data[0];
    }

    void print()
    {
        Serial.printf("Dart Time packet: \n");
        Serial.printf("Dart Remaining Time: %u\n", dart_remaining_time);
    }
};

/// @brief Robot performance system data. Transmitted at 10Hz to single robot
/// @note ID: 0x0201
struct RobotStatus
{
    /// @brief Robot ID
    uint8_t robot_id = 0;
    /// @brief Robot Level [1-3]
    uint8_t robot_level = 0;
    /// @brief Current health
    uint16_t robot_HP = 0;
    /// @brief Max health
    uint16_t robot_max_HP = 0;

    /// @brief Robot 1 17mm barrel cooling rate (u/s)
    uint16_t robot_1_17mm_barrel_cooling_rate = 0;
    /// @brief Robot 1 17mm barrel heat limit
    uint16_t robot_1_17mm_barrel_heat_limit = 0;
    /// @brief Robot 1 17mm Initial Launch Speed Limit (m/s)
    uint16_t robot_1_17mm_initial_launching_speed_limit = 0;

    /// @brief Robot 2 17mm barrel cooling rate (u/s)
    uint16_t robot_2_17mm_barrel_cooling_rate = 0;
    /// @brief Robot 2 17mm barrel heat limit
    uint16_t robot_2_17mm_barrel_heat_limit = 0;
    /// @brief Robot 2 17mm Initial Launch Speed Limit (m/s)
    uint16_t robot_2_17mm_initial_launching_speed_limit = 0;

    /// @brief Robot 1 42mm barrel cooling rate (u/s)
    uint16_t robot_1_42mm_barrel_cooling_rate = 0;
    /// @brief Robot 1 42mm barrel heat limit
    uint16_t robot_1_42mm_barrel_heat_limit = 0;
    /// @brief Robot 1 42mm Initial Launch Speed Limit (m/s)
    uint16_t robot_1_42mm_initial_launching_speed_limit = 0;

    /// @brief Robot Chassis Power Consumption Limit
    uint16_t chassis_power_limit = 0;
    /// @brief Whether gimbol is powered with 24V
    uint8_t is_gimbol_powered = 0;
    /// @brief Whether chassis is powered with 24V
    uint8_t is_chassis_powered = 0;
    /// @brief Whether shooter is powered with 24V
    uint8_t is_shooter_powered = 0;

    void InitializeFromRaw(FrameData& data)
    {
        robot_id = data.data[0];
        robot_level = data.data[1];
        robot_HP = (data.data[3] << 8) | data.data[2];
        robot_max_HP = (data.data[5] << 8) | data.data[4];

        robot_1_17mm_barrel_cooling_rate = (data.data[7] << 8) | data.data[6];
        robot_1_17mm_barrel_heat_limit = (data.data[9] << 8) | data.data[8];
        robot_1_17mm_initial_launching_speed_limit = (data.data[11] << 8) | data.data[10];

        robot_2_17mm_barrel_cooling_rate = (data.data[13] << 8) | data.data[12];
        robot_2_17mm_barrel_heat_limit = (data.data[15] << 8) | data.data[14];
        robot_2_17mm_initial_launching_speed_limit = (data.data[17] << 8) | data.data[16];

        robot_1_42mm_barrel_cooling_rate = (data.data[19] << 8) | data.data[18];
        robot_1_42mm_barrel_heat_limit = (data.data[21] << 8) | data.data[20];
        robot_1_42mm_initial_launching_speed_limit = (data.data[23] << 8) | data.data[22];

        chassis_power_limit = (data.data[25] << 8) | data.data[24];

        is_gimbol_powered = data.data[26] & 0x1;
        is_chassis_powered = (data.data[26] & 0x2) >> 1;
        is_shooter_powered = (data.data[26] & 0x4) >> 2;
    }

    void print()
    {
        Serial.printf("Robot Status: \n");
        Serial.printf("Robot ID: %d\n", robot_id);
        Serial.printf("Robot level: %d\n", robot_level);
        Serial.printf("Robot HP: %d\n", robot_HP);
        Serial.printf("Robot max HP: %d\n", robot_max_HP);

        Serial.printf("Robot 1 17mm cooling rate: %d\n", robot_1_17mm_barrel_cooling_rate);
        Serial.printf("Robot 1 17mm heat limit: %d\n", robot_1_17mm_barrel_heat_limit);
        Serial.printf("Robot 1 17mm launching speed: %d\n", robot_1_17mm_initial_launching_speed_limit);
        Serial.printf("Robot 2 17mm cooling rate: %d\n", robot_2_17mm_barrel_cooling_rate);
        Serial.printf("Robot 2 17mm heat limit: %d\n", robot_2_17mm_barrel_heat_limit);
        Serial.printf("Robot 2 17mm launching speed: %d\n", robot_2_17mm_initial_launching_speed_limit);
        Serial.printf("Robot 42mm cooling rate: %d\n", robot_1_42mm_barrel_cooling_rate);
        Serial.printf("Robot 42mm heat limit: %d\n", robot_1_42mm_barrel_heat_limit);
        Serial.printf("Robot 42mm launching speed: %d\n", robot_1_42mm_initial_launching_speed_limit);

        Serial.printf("Chassis Power Limit: %d\n", chassis_power_limit);
        Serial.printf("Is Gimbol Powered: %d\n", is_gimbol_powered);
        Serial.printf("Is Chassis Powered: %d\n", is_chassis_powered);
        Serial.printf("Is Shooter Powered: %d\n", is_shooter_powered);
    }
};

/// @brief Provides real-time power and heat data. Transmitted at 50Hz to single robot
/// @note ID: 0x0202
struct HeatPowerData
{
    /// @brief Chassis port output voltage (mV)
    uint16_t chassis_voltage = 0;
    /// @brief Chassis port output current (mA)
    uint16_t chassis_ampage = 0;
    /// @brief Chassis power (W)
    float chassis_power = 0.f;

    /// @brief Buffer energy (J)
    uint16_t buffer_energy = 0;
    /// @brief Barrel heat of the 1st 17mm 
    uint16_t heat_17mm_1_barrel = 0;
    /// @brief Barrel heat of the 2nd 17mm
    uint16_t heat_17mm_2_barrel = 0;
    /// @brief Barrel heat of the 42mm
    uint16_t heat_42mm_barrel = 0;

    void InitializeFromRaw(FrameData& data)
    {
        uint64_t rawFloat = 0;

        chassis_voltage = (data.data[1] << 8) | data.data[0];
        chassis_ampage = (data.data[3] << 8) | data.data[2];

        rawFloat = data.data[7];
        rawFloat = (rawFloat << 8) | data.data[6];
        rawFloat = (rawFloat << 8) | data.data[5];
        rawFloat = (rawFloat << 8) | data.data[4];
        assert(sizeof(float) == sizeof(uint32_t));
        memcpy(&chassis_power, &rawFloat, sizeof(float));

        buffer_energy = (data.data[9] << 8) | data.data[8];
        heat_17mm_1_barrel = (data.data[11] << 8) | data.data[10];
        heat_17mm_2_barrel = (data.data[13] << 8) | data.data[12];
        heat_42mm_barrel = (data.data[15] << 8) | data.data[14];
    }

    void print()
    {
        Serial.printf("HeatPowerData packet: \n");
        Serial.printf("Chassis Voltage: %d\n", chassis_voltage);
        Serial.printf("Chassis Ampage: %d\n", chassis_ampage);
        Serial.printf("Chassis Power: %f\n", chassis_power);
        Serial.printf("Buffer Energy: %d\n", buffer_energy);
        Serial.printf("17mm 1 Heat: %d\n", heat_17mm_1_barrel);
        Serial.printf("17mm 2 Heat: %d\n", heat_17mm_2_barrel);
        Serial.printf("42mm Heat: %d\n", heat_42mm_barrel);
    }
};

/// @brief Provides absolute GPS position. Transmitted at 10Hz to single robot
/// @note ID: 0x0203
struct RobotPosition
{
    /// @brief X position of the robot (in meters)
    float x = 0.f;
    /// @brief Y position of the robot (in meters)
    float y = 0.f;
    /// @brief Z position of the robot (in meters)
    float z = 0.f;
    /// @brief Angle of the robot (yaw) (in degrees)
    float angle = 0.f;

    void InitializeFromRaw(FrameData& data)
    {
        uint64_t rawFloat = 0;
        int i = 0;
        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        assert(sizeof(float) == sizeof(uint32_t));
        memcpy(&x, &rawFloat, sizeof(float));

        i++;
        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        assert(sizeof(float) == sizeof(uint32_t));
        memcpy(&y, &rawFloat, sizeof(float));

        i++;
        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        assert(sizeof(float) == sizeof(uint32_t));
        memcpy(&z, &rawFloat, sizeof(float));

        i++;
        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        assert(sizeof(float) == sizeof(uint32_t));
        memcpy(&angle, &rawFloat, sizeof(float));
    }

    void print()
    {
        Serial.printf("Robot Position: \n");
        Serial.printf("X: %f\n", x);
        Serial.printf("Y: %f\n", y);
        Serial.printf("Z: %f\n", z);
        Serial.printf("Angle: %f\n", angle);
    }
};

/// @brief Robot buff data. Transmitted at 3Hz to single robot
/// @note ID: 0x0204
struct BuffData
{
    /// @brief Robot’s HP Recovery Buff (in percentage; a value of 10 means the maximum HP recovery per second is 10 %)
    uint8_t recovery_buff = 0;
    /// @brief Robot barrel cooling rate (in absolute value; a value of 5 means a cooling rate of 5 times)
    uint8_t cooling_buff = 0;
    /// @brief Robot defense buff (in percentage; a value of 50 means a defense buff of 50%)
    uint8_t defense_buff = 0;
    /// @brief Robot attack buff (in percentage; a value of 50 means an attack buff of 50%)
    uint16_t attack_buff = 0;

    void InitializeFromRaw(FrameData& data)
    {
        recovery_buff = data[0];
        cooling_buff = data[1];
        defense_buff = data[2];
        attack_buff = (data[4] << 8) | data[3];
    }

    void print()
    {
        Serial.printf("Robot Buff Data packet: \n");
        Serial.printf("Recovery Buff: %u%\n", recovery_buff);
        Serial.printf("Cooling Rate Buff: %ux\n", cooling_buff);
        Serial.printf("Defense Buff: %u%\n", defense_buff);
        Serial.printf("Attack Buff: %u%\n", attack_buff);
    }
};

/// @brief Air support time data. Transmitted at 10Hz to all our aerial robots
/// @note ID: 0x0205
struct AirSupportData
{
    /// @brief Aerial robot status (0 = cooling, 1 = cooling is completed, 2 = air support is ongoing)
    uint8_t airforce_status = 0;
    /// @brief The remaining time of this status (in seconds (floored to nearest int))
    uint8_t time_remaining = 0;

    void InitializeFromRaw(FrameData& data)
    {
        airforce_status = data[0];
        time_remaining = data[1];
    }

    void print()
    {
        Serial.printf("Air Support Data packet: \n");
        Serial.printf("Status: %x\n", airforce_status);
        Serial.printf("Time Remaining: %us\n", time_remaining);
    }
};

/// @brief Damage status data. Transmitted on damage event to single robot
/// @note ID: 0x0206
struct HurtData
{
    /// @brief the ID of which armor module was hit, 0 if hurt by other means
    uint8_t armor_id = 0;
    /// @brief Type of HP changes (0 = projectiles, 1 = a critical module goes offline, 2 = shooting to fast (speed), 3 = barrel heat, 4 = power consumption, 5 = collision)
    uint8_t deduction_reason = 0;

    void InitializeFromRaw(FrameData& data)
    {
        armor_id = data[0] & 0xf;
        deduction_reason = (data[0] & 0xf0) >> 4;
    }

    void print()
    {
        Serial.printf("Hurt Data packet: \n");
        Serial.printf("Armor ID Hit: %u\n", armor_id);
        Serial.printf("Deduction Reason: %u\n", deduction_reason);
    }
};

/// @brief Real-time launching data. Transmitted on shoot event to single robot
/// @note ID: 0x0205
struct ShootData
{
    /// @brief Projectile time (1 = 17mm, 2 = 42mm)
    uint8_t bullet_type = 0;
    /// @brief Launching mechanism ID (1 = first 17mm, 2 = second 17mm, 3 = 42mm)
    uint8_t shooter_number = 0;
    /// @brief Projectile launch speed (unit: Hz)
    uint8_t fire_rate = 0;
    /// @brief Projectile initial speed (unit: m/s)
    float initial_speed = 0.f;

    void InitializeFromRaw(FrameData& data)
    {
        bullet_type = data[0];
        shooter_number = data[1];
        fire_rate = data[2];

        uint64_t rawFloat = 0;
        rawFloat = data.data[6];
        rawFloat = (rawFloat << 8) | data.data[5];
        rawFloat = (rawFloat << 8) | data.data[4];
        rawFloat = (rawFloat << 8) | data.data[3];
        assert(sizeof(float) == sizeof(uint32_t));
        memcpy(&initial_speed, &rawFloat, sizeof(float));
    }

    void print()
    {
        Serial.printf("Shooting Data packet: \n");
        Serial.printf("Projectile Type: %x\n", bullet_type);
        Serial.printf("Shooter ID: %x\n", shooter_number);
        Serial.printf("Fire Rate: %uHz\n", fire_rate);
        Serial.printf("Projectile Initial Speed: %fm/s\n", initial_speed);
    }
};

/// @brief Projectile Allowance data. Transmitted at 10Hz to all our robots that shoot
/// @note ID: 0x0208
struct ProjectileAllowance
{
    /// @brief 17mm projectile allowance
    uint16_t allowance_17mm = 0;
    /// @brief 42mm projectile allowance
    uint16_t allowance_42mm = 0;
    /// @brief Number of remaining coins
    uint16_t allowance_gold_coin = 0;

    void InitializeFromRaw(FrameData& data)
    {
        allowance_17mm = (data[1] << 8) | data[0];
        allowance_42mm = (data[3] << 8) | data[2];
        allowance_gold_coin = (data[5] << 8) | data[4];
    }

    void print()
    {
        Serial.printf("Projectile Allowance packet: \n");
        Serial.printf("17mm Count: %u\n", allowance_17mm);
        Serial.printf("42mm Count: %u\n", allowance_42mm);
        Serial.printf("Gold Coinds: %u\n", allowance_gold_coin);
    }
};

/// @brief Robot RFID status. Transmitted at 3Hz to all own robots with RFID module
/// @todo seperate data into seperate boolean vals
/// @note ID: 0x0209
struct RFIDData
{
    uint32_t rfid_status = 0;
    void InitializeFromRaw(FrameData& data)
    {
        Serial.println("TODO: dont forget to implement this");
    }

    void print()
    {
        Serial.println("TODO: dont forget to implement this");
    }
};

/// @brief Dart player client command data. Transmitted at 10Hz to all our dart robots
/// @note ID: 0x020A
struct DartClientCommand
{
    /// @brief Current status of the Dart Launching Station (1 = closed, 2 = opening or closing, 0 = open)
    uint8_t dart_launch_opening_status = 0;
    /// @brief Dart target: (Outpost as default) (0 = outpost, 1 = base)
    uint8_t dart_attack_target = 0;
    /// @brief Time remaining in the competition when switching target (seconds)
    uint16_t target_change_time = 0;
    /// @brief Time remaining in the competition when the Operator confirms the launch command for the last time (seconds)
    uint16_t latest_launch_cmd_time = 0;

    void InitializeFromRaw(FrameData& data)
    {
        dart_launch_opening_status = data[0];
        dart_attack_target = data[1];

        target_change_time = (data[3] << 8) | data[2];
        latest_launch_cmd_time = (data[5] << 8) | data[4];
    }

    void print()
    {
        Serial.printf("Dart Client Command packet: \n");
        Serial.printf("Opening status: %x\n", dart_launch_opening_status);
        Serial.printf("Attack Target: %x\n", dart_attack_target);
        Serial.printf("Time of Target Swap: %us\n", target_change_time);
        Serial.printf("Time of Launch Confimation: %us\n", latest_launch_cmd_time);
    }
};

/// @brief Ground Robot position data. Transmitted at 1Hz to all our ground robots
/// @note ID: 0x020B
struct GroundRobotPositions
{
    float hero_x = 0.f;
    float hero_y = 0.f;
    float engineer_x = 0.f;
    float engineer_y = 0.f;
    float standard_3_x = 0.f;
    float standard_3_y = 0.f;
    float standard_4_x = 0.f;
    float standard_4_y = 0.f;
    float standard_5_x = 0.f;
    float standard_5_y = 0.f;

    void InitializeFromRaw(FrameData& data)
    {
        uint64_t rawFloat = 0;
        int i = 0;

        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        assert(sizeof(float) == sizeof(uint32_t));
        memcpy(&hero_x, &rawFloat, sizeof(float));

        i++;
        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        assert(sizeof(float) == sizeof(uint32_t));
        memcpy(&hero_y, &rawFloat, sizeof(float));

        i++;
        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        assert(sizeof(float) == sizeof(uint32_t));
        memcpy(&engineer_x, &rawFloat, sizeof(float));

        i++;
        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        assert(sizeof(float) == sizeof(uint32_t));
        memcpy(&engineer_y, &rawFloat, sizeof(float));

        i++;
        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        assert(sizeof(float) == sizeof(uint32_t));
        memcpy(&standard_3_x, &rawFloat, sizeof(float));

        i++;
        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        assert(sizeof(float) == sizeof(uint32_t));
        memcpy(&standard_3_y, &rawFloat, sizeof(float));

        i++;
        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        assert(sizeof(float) == sizeof(uint32_t));
        memcpy(&standard_4_x, &rawFloat, sizeof(float));

        i++;
        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        assert(sizeof(float) == sizeof(uint32_t));
        memcpy(&standard_4_y, &rawFloat, sizeof(float));

        i++;
        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        assert(sizeof(float) == sizeof(uint32_t));
        memcpy(&standard_5_x, &rawFloat, sizeof(float));

        i++;
        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        assert(sizeof(float) == sizeof(uint32_t));
        memcpy(&standard_5_y, &rawFloat, sizeof(float));
    }

    void print()
    {
        Serial.printf("Ground Robot Position Data packet: \n");
        Serial.printf("Hero x: %fm\ty: %fm\n", hero_x, hero_y);
        Serial.printf("Engineer x: %fm\ty: %fm\n", engineer_x, engineer_y);
        Serial.printf("Standard 3 x: %fm\ty: %fm\n", standard_3_x, standard_3_y);
        Serial.printf("Standard 4 x: %fm\ty: %fm\n", standard_4_x, standard_4_y);
        Serial.printf("Standard 5 x: %fm\ty: %fm\n", standard_5_x, standard_5_y);
    }
};

/// @brief cmdID: 0x020C
/// @brief Radar marked progress data. Transmitted at 1Hz to our radar robots
struct RadarMarkData
{
    /// @brief Marked progress of opponent’s Hero Robot: 0-120
    uint8_t mark_hero_progress = 0;
    /// @brief Marked progress of opponent’s Engineer Robot: 0-120
    uint8_t mark_engineer_progress = 0;
    /// @brief Marked progress of opponent’s Standard Robot No. 3: 0-120
    uint8_t mark_standard_3_progress = 0;
    /// @brief Marked progress of opponent’s Standard Robot No. 4: 0-120
    uint8_t mark_standard_4_progress = 0;
    /// @brief Marked progress of opponent’s Standard Robot No. 5: 0-120
    uint8_t mark_standard_5_progress = 0;
    /// @brief Marked progress of opponent’s Sentry Robot: 0-120
    uint8_t mark_sentry_progress = 0;

    void InitializeFromRaw(FrameData& data)
    {
        mark_hero_progress = data[0];
        mark_engineer_progress = data[1];
        mark_standard_3_progress = data[2];
        mark_standard_4_progress = data[3];
        mark_standard_5_progress = data[4];
        mark_sentry_progress = data[5];
    }

    void print()
    {
        Serial.printf("Radar Mark Data packet: \n");
        Serial.printf("Hero Progress: %u\n", mark_hero_progress);
        Serial.printf("Engineer Progress: %u\n", mark_engineer_progress);
        Serial.printf("Standard 3 Progress: %u\n", mark_standard_3_progress);
        Serial.printf("Standard 4 Progress: %u\n", mark_standard_4_progress);
        Serial.printf("Standard 5 Progress: %u\n", mark_standard_5_progress);
        Serial.printf("Sentry Progress: %u\n", mark_sentry_progress);
    }
};

// TODO: implement 0x0300s robot interaction data

/// @brief A complete RefSystem Frame, contains the Header, CommandID, Data, and CRC16 protions
struct Frame
{
    /// @brief Header portion of the complete Frame
    FrameHeader header{};
    /// @brief Command ID portion of the complete Frame
    uint16_t cmdID = 0;
    /// @brief Data portion of the complete Frame
    FrameData data{};
    /// @brief 16-bit CRC portion of the complete Frame
    uint16_t CRC16 = 0;

    void printRaw()
    {
        Serial.printf("New Frame Read\n");
        Serial.printf("Command: %.4x\n", cmdID);

        Serial.println("Frame Data: ");
        for (int i = 0; i < header.data_length; i++)
        {
            Serial.printf("%.2x ", data.data[i]);
            if (i % 16 == 15)
                Serial.println();
        }
        Serial.println();

        Serial.printf("Ending CRC16: %.4x\n", CRC16);
    }

    void printType()
    {
        Serial.printf("Received Frame: %.4x\n", cmdID);
    }

    void print()
    {
        switch (cmdID)
        {
        case FrameType::GAME_STATUS:
        {
            GameStatus frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::GAME_RESULT:
        {
            GameResult frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::ROBOT_HEALTH:
        {
            RobotHealth frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::EVENT_DATA:
        {
            EventData frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::PROJECTILE_SUPPLY_STATUS:
        {
            ProjectileSupplyStatus frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::REFEREE_WARNING:
        {
            RefereeWarning frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::DART_TIME:
        {
            DartTime frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::ROBOT_STATUS:
        {
            RobotStatus frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::HEAT_POWER_DATA:
        {
            HeatPowerData frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::ROBOT_POSITION:
        {
            RobotPosition frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::BUFF_DATA:
        {
            BuffData frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::AIR_SUPPORT_DATA:
        {
            AirSupportData frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::SHOOT_DATA:
        {
            ShootData frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::PROJECTILE_ALLOWANCE:
        {
            ProjectileAllowance frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::RFID_DATA:
        {
            RFIDData frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::DART_CLIENT_COMMAND:
        {
            DartClientCommand frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::GROUND_ROBOT_POSITIONS:
        {
            GroundRobotPositions frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        case FrameType::RADAR_MARK_DATA:
        {
            RadarMarkData frame;
            frame.InitializeFromRaw(data);
            frame.print();
            break;
        }
        default:
            Serial.println("Invalid Frame ID");
            break;
        }
        Serial.println();
    }
};



#endif // REF_SYSTEM_PACKET_DEFINITIONS_HPP