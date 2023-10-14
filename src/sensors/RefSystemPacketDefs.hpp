#ifndef REF_SYSTEM_PACKET_DEFINITIONS_HPP
#define REF_SYSTEM_PACKET_DEFINITIONS_HPP

#include <string>

constexpr uint16_t REF_MAX_FRAME_DATA_SIZE = 128;
constexpr uint16_t REF_MAX_COMMAND_ID = 0x0307;

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

const char* get_frame_name(FrameType&)

struct FrameHeader
{
    static const uint8_t size = 5;

    uint8_t SOF = 0;
    uint16_t data_length = 0;
    uint8_t seq = 0;
    uint8_t CRC8 = 0;
};

struct FrameData
{
    uint8_t data[REF_MAX_FRAME_DATA_SIZE] = { 0 };
};

struct Frame
{
    FrameHeader header{};
    uint16_t cmdID = 0;
    FrameData data{};
    uint16_t CRC16 = 0;

    void print()
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
};

/// @brief Game result data. Transmitted at the end of the competition
/// @note ID: 0x0002
struct GameResult
{
    /// @brief Winner of the game (0: Draw, 1: Red, 2: Blue)
    uint8_t winner = 0;
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
};

/// @brief Site Event Data. Transmitted at 3Hz to all of our robots
/// @note ID: 0x0101
struct EventData
{
    /// @brief Site data of our own objects/flags
    /// @todo seperate all of these values into individual variables
    uint32_t event_data = 0;
};

/// @brief Official Projectile Supplier action identifier data. Transmitted only when Supplier releases projectiles. Sent to all of our robots
/// @note ID: 0x0102
struct ProjectileSupplyStatus
{
    uint8_t supply_projectile_id = 0;
    uint8_t supply_robot_id = 0;
    uint8_t supply_projectile_status = 0;
    uint8_t supply_projectile_count = 0;
};

/// @brief Referee warning data. Transmitted if any warnings are issued to our team (all of our robots)
/// @note ID: 0x0104
struct RefereeWarning
{
    /// @brief Penalty level (1: Yellow, 2: Red, 3: Forfeiture)
    uint8_t level = 0;
    /// @brief ID of robot being issued a penalty
    uint8_t offending_robot_id = 0;
};

/// @brief Dart launching time data. Transmitted at 3Hz to all of our team's robots
/// @note ID: 0x0105
struct DartTime
{
    /// @brief Own team’s remaining time for dart launching (in seconds)
    uint8_t dart_remaining_time = 0;
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
        chassis_power = *reinterpret_cast<float*>(&rawFloat);

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
        x = *reinterpret_cast<float*>(&rawFloat);

        i++;
        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        y = *reinterpret_cast<float*>(&rawFloat);

        i++;
        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        z = *reinterpret_cast<float*>(&rawFloat);

        i++;
        rawFloat = data.data[3 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[2 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[1 + i * 4];
        rawFloat = (rawFloat << 8) | data.data[0 + i * 4];
        angle = *reinterpret_cast<float*>(&rawFloat);
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

/// @brief cmdID: 0x0204
struct BuffData
{
    uint8_t recovery_buff = 0;
    uint8_t cooling_buff = 0;
    uint8_t defence_buff = 0;
    uint16_t attack_buff = 0;
};

/// @brief cmdID: 0x0205
struct AirSupportData
{
    uint8_t airforce_status = 0;
    uint8_t time_remaining = 0;
};

/// @brief cmdID: 0x0206
struct HurtData
{
    uint8_t armor_id = 0;
    uint8_t deduction_reason = 0;
};

/// @brief cmdID: 0x0207
struct ShootData
{
    uint8_t bullet_type = 0;
    uint8_t shooter_number = 0;
    uint8_t fire_rate = 0;
    float initial_speed = 0.f;
};

/// @brief cmdID: 0x0208
struct ProjectileAllowance
{
    uint16_t allowance_17mm = 0;
    uint16_t allowance_42mm = 0;
    uint16_t allowance_gold_coin = 0;
};

/// @brief cmdID: 0x0209
struct RFIDData
{
    uint32_t rfid_status = 0;
};

/// @brief cmdID: 0x020A
struct DartClientCommand
{
    uint8_t dart_launch_opening_status = 0;
    uint8_t dart_attack_target = 0;
    uint16_t target_change_time = 0;
    uint16_t latest_launch_cmd_time = 0;
};

/// @brief cmdID: 0x020B
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
};

/// @brief cmdID: 0x020C
struct RadarMarkData
{
    uint8_t mark_hero_progress = 0;
    uint8_t mark_engineer_progress = 0;
    uint8_t mark_standard_3_progress = 0;
    uint8_t mark_standard_4_progress = 0;
    uint8_t mark_standard_5_progress = 0;
    uint8_t mark_sentry_progress = 0;
};

// TODO: implement 0x0300s robot interaction data

#endif // REF_SYSTEM_PACKET_DEFINITIONS_HPP