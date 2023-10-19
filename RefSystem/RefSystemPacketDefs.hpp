#ifndef REF_SYSTEM_PACKET_DEFINITIONS_HPP
#define REF_SYSTEM_PACKET_DEFINITIONS_HPP

constexpr uint16_t REF_MAX_PACKET_SIZE = 196;
constexpr uint16_t REF_MAX_COMMAND_ID = 0x0307;

/*--- Ref System Frame Structs ---*/

enum FrameType
{
    GAME_STATUS = 0x0001,
    GAME_RESULT = 0x0002,
    ROBOT_HEALTH = 0x0003
};

struct FrameHeader
{
    static const uint8_t packet_size = 5;

    /// @brief Start of Frame byte, should be 0xAF if valid frame
    uint8_t SOF = 0;
    /// @brief length of the FrameData portion of a Frame
    uint16_t data_length = 0;
    /// @brief Sequence number, increments per frame, wraps around 255
    uint8_t sequence = 0;
    /// @brief An 8-bit CRC for the FrameHeader only
    uint8_t CRC = 0;

    void print()
    {
        Serial.printf("\tSOF: %x\n", SOF);
        Serial.printf("\tLength: %u\n", data_length);
        Serial.printf("\tSequence: %u\n", sequence);
        Serial.printf("\tCRC: %x\n", CRC);
    }
};

struct FrameData
{
    /// @brief Data array to hold the Frame data portion
    uint8_t data[REF_MAX_PACKET_SIZE] = { 0 };

    /// @brief Helpful index operator. Allows array-like indexing from the object itself
    uint8_t operator[](int index)
    {
        return data[index];
    }
};

struct Frame
{
    /// @brief Header portion of a Frame
    FrameHeader header{};
    /// @brief Command ID potion of a Frame
    uint16_t commandID = 0;
    /// @brief Data portion of a Frame
    FrameData data{};
    /// @brief 16-bit CRC for the entire Frame
    uint16_t CRC = 0;

    void print()
    {
        Serial.println("Read Frame:");
        header.print();
        Serial.printf("Command ID: %.2x\n", commandID);
        for (int i = 0; i < header.data_length; i++)
        {
            Serial.printf("%x ", data[i]);
        }
        Serial.println();
        Serial.printf("CRC: %.2x\n", CRC);
    }
};

/*--- Ref System Command ID Packet Structs ---*/

/// @brief Competition status data. Transmitted at 3Hz to all robots
/// @note ID: 0x0001
struct GameStatus
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 11;

    /// @brief [0] = Competition type | [1] = Competition stage \
    /// @brief Competition Type 1 = RMUC, 2 = RMUTC, 3 = RMUAIC, 4 = RMUL (3v3), 5 = RMUL (1v1) \
    /// @brief Competition Stage 0 = Pre-comp, 1 = Setup, 2 = Initialization, 3 = 5-seconds, 4 = Ongoing, 5 = Calculating results
    uint8_t game_config[2] = { 0 };
    /// @brief Remaining time in the current round; unit: second
    uint16_t round_timer = 0;
    /// @brief UNIX time, effective after the robot is correctly linked to the Referee System’s NTP server
    uint64_t real_time = 0;

    void initialize_from_data(FrameData& data)
    {
        game_config[0] = data[0] & 0x0f;
        game_config[1] = (data[0] & 0xf0) >> 4;
        round_timer = (data[2] << 8) | data[1];
        real_time = data[10];
        real_time = (real_time << 8) | data[9];
        real_time = (real_time << 8) | data[8];
        real_time = (real_time << 8) | data[7];
        real_time = (real_time << 8) | data[6];
        real_time = (real_time << 8) | data[5];
        real_time = (real_time << 8) | data[4];
        real_time = (real_time << 8) | data[3];
    }

    void print()
    {
        Serial.println("--==GameStatus Frame==--");
        Serial.printf("Game Type: %x\n", game_config[0]);
        Serial.printf("Game Stage: %x\n", game_config[1]);
        Serial.printf("Round Time: %us\n", round_timer);
        Serial.printf("Real Time: %u\n", real_time);
    }
};

/// @brief Competition result data. Transmitted on game end event to all robots
/// @note ID: 0x0002
struct GameResult
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 1;

    /// @brief Winner of the match \
    /// @brief 0 = Draw, 1 = Red, 2 = Blue
    uint8_t winner = 0;

    void initialize_from_data(FrameData& data)
    {
        winner = data[0];
    }

    void print()
    {
        Serial.println("--== GameResult Frame ==--");
        Serial.printf("Match Winner: %x\n", winner);
    }
};

/// @brief Robot health data. Transmitted at 3Hz to all robots
/// @note ID: 0x0003
struct RobotHealth
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 32;

    /// @brief Robot health array for Red Team's robots \
    /// @brief [0] = Hero \
    /// @brief [1] = Engineer \
    /// @brief [2/3/4] = Standard \
    /// @brief [5] = Sentry \
    /// @brief [6] = Outpost \
    /// @brief [7] = Base
    uint16_t red_robot_HP[8] = { 0 };

    /// @brief Robot health array for Blue Team's robots \
    /// @brief [0] = Hero \
    /// @brief [1] = Engineer \
    /// @brief [2/3/4] = Standard \
    /// @brief [5] = Sentry \
    /// @brief [6] = Outpost \
    /// @brief [7] = Base    
    uint16_t blue_robot_HP[8] = { 0 };

    void initialize_from_data(FrameData& data)
    {
        for (int i = 0; i < 8; i++)
        {
            red_robot_HP = (data[i * 2 + 1] << 8) | data[i * 2];
        }

        for (int i = 8; i < 16; i++)
        {
            blue_robot_HP = (data[i * 2 + 1] << 8) | data[i * 2];
        }
    }

    void print()
    {
        Serial.println("--== RobotHealth Frame ==--");
        Serial.printf("Red Hero: %u\n", red_robot_HP[0]);
        Serial.printf("Red Engineer: %u\n", red_robot_HP[1]);
        Serial.printf("Red Standard 3: %u\n", red_robot_HP[2]);
        Serial.printf("Red Standard 4: %u\n", red_robot_HP[3]);
        Serial.printf("Red Standard 5: %u\n", red_robot_HP[4]);
        Serial.printf("Red Sentry: %u\n", red_robot_HP[5]);
        Serial.printf("Red Outpost: %u\n", red_robot_HP[6]);
        Serial.printf("Red Base: %u\n", red_robot_HP[7]);
        Serial.printf("Blue Hero: %u\n", blue_robot_HP[0]);
        Serial.printf("Blue Engineer: %u\n", blue_robot_HP[1]);
        Serial.printf("Blue Standard 3: %u\n", blue_robot_HP[2]);
        Serial.printf("Blue Standard 4: %u\n", blue_robot_HP[3]);
        Serial.printf("Blue Standard 5: %u\n", blue_robot_HP[4]);
        Serial.printf("Blue Sentry: %u\n", blue_robot_HP[5]);
        Serial.printf("Blue Outpost: %u\n", blue_robot_HP[6]);
        Serial.printf("Blue Base: %u\n", blue_robot_HP[7]);
    }
};

/// @brief Site event data. Transmitted at 3Hz to all our robots
/// @note ID: 0x0101
struct SiteEvent
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 4;

    /// @brief Whether this Restoration Zone is occupied or not \
    /// @brief [0] = Zone in front of supplier \
    /// @brief [1] = Zone left of supplier \
    /// @brief [2] = Zone right of supplier
    uint8_t restoration_zone_occupation[3] = { 0 };

    /// @brief Status of our own Power Runes \
    /// @brief [0] = Occupation of our Activation Point \
    /// @brief [1] = Activation of Small Rune \
    /// @brief [2] = Activation of Large Rune
    uint8_t power_rune_status[3] = { 0 };

    /// @brief Occupation of different Elevated Ground types \
    /// @brief [0] = Ring-Shaped \
    /// @brief [1] = R3 Trapezoid \
    /// @brief [2] = R4 Trapezoid
    uint8_t elevated_ground_occupation[3] = { 0 };

/// @brief Value of own Base’s Virtual Shield (0-250)
    uint8_t base_shield_HP = 0;
    /// @brief HP of own Outpost (0-1500)
    uint16_t outpost_HP = 0;
    /// @brief Whether the Sentry is in own Patrol Zone
    uint8_t is_sentry_in_patrol_zone = 0;

    void initialize_from_data(FrameData& data)
    {

    }

    void print()
    {

    }
};

/// @brief Official Projectile Supplier action identifier data.Transmitted on Projectile Supplier event to all our robots
/// @note ID: 0x0102
struct ProjectileSupplier
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 4;

    /// @brief IDs of the Supplier and Robot \
    /// @brief [0] = Supplier Outlet ID \
    /// @brief [1] = Reloading robot ID \
    /// @brief Reloading Robot ID: 0 = None, 0xx = red robot, 1xx = blue robot
    uint8_t corresponding_ID[2] = { 0 };

    /// @brief Status of the Projectile Outlet \
    /// @brief 0 = Closed, 1 = Preparing, 2 = Releasing
    uint8_t outlet_status = 0;

    /// @brief Number of supplied projectiles
    uint8_t supplied_count = 0;

    void initialize_from_data(FrameData& data)
    {

    }

    void print()
    {

    }
};

/// @brief Referee warning data. Transmitted on penalty event to all robots on penalized team
/// @note ID: 0x0104
struct RefereeWarning
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 2;

    /// @brief  Penalty Level \
    /// @brief 1 = Yellow Card, 2 = Red Card, 3 = Forfeiture
    uint8_t penalty_level = 0;

    /// @brief Offending robot ID \
    /// @brief 0xx = Red Team, 1xx = Blue Team
    /// @note In the case of a forfeiture or where both teams have been issued a Yellow Card, the value is 0.
    uint8_t robot_ID = 0;

    void initialize_from_data(FrameData& data)
    {

    }

    void print()
    {

    }
};

/// @brief Dart launching time data. Transmitted at 10Hz to all our robots
/// @note ID: 0x0105
struct DartLaunch
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 1;

    /// @brief Own team’s remaining time for dart launching; unit: second
    uint8_t dart_remaining_time = 0;

    void initialize_from_data(FrameData& data)
    {

    }

    void print()
    {

    }
};

/// @brief Robot performance system data. Transmitted at 10Hz to single robot
/// @note ID: 0x0201
struct RobotPerformance
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 27;

    /// @brief Current robot ID
    uint8_t robot_ID = 0;
    /// @brief Current robot level
    uint8_t robot_level = 0;

    /// @brief Current robot HP status \
    /// @brief [0] = current HP, [1] = maximum HP
    uint16_t health_status[2] = { 0 };

    /// @brief Robot 1 17mm barrel status \
    /// @brief [0] = barrel cooling value per second \
    /// @brief [1] = barrel heat limit \
    /// @brief [2] = Launching Mechanism’s Initial Launch Speed Limit (unit: m/s)
    uint16_t barrel_17mm_1_status[3] = { 0 };
    /// @brief Robot 2 17mm barrel status \
    /// @brief [0] = barrel cooling value per second \
    /// @brief [1] = barrel heat limit \
    /// @brief [2] = Launching Mechanism’s Initial Launch Speed Limit (unit: m/s)
    uint16_t barrel_17mm_2_status[3] = { 0 };
    /// @brief Robot 42mm barrel status \
    /// @brief [0] = barrel cooling value per second \
    /// @brief [1] = barrel heat limit \
    /// @brief [2] = Launching Mechanism’s Initial Launch Speed Limit (unit: m/s)
    uint16_t barrel_42mm_status[3] = { 0 };

    /// @brief Robot Chassis Power Consumption Limit
    uint16_t chassis_power_limit = 0;
    /// @brief Power Management Module output status. If true, 24V output \
    /// @brief [0] = Output from gimbal port \
    /// @brief [1] = Output from chassis port \
    /// @brief [2] = Output from shooter port
    uint8_t power_output_status[3] = { 0 };

    void initialize_from_data(FrameData& data)
    {

    }

    void print()
    {

    }
};

/// @brief Real-time power heat data. Transmitted at 50Hz to single robot
/// @note ID: 0x0202
struct PowerHeat
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 16;

    /// @brief Power Management Module chassis port output \
    /// @brief [0] = voltage (mV) \
    /// @brief [1] = current (mA) 
    uint16_t chassis_power_status[2] = { 0 };

    /// @brief Chassis Power (unit: W)
    float chassis_power = 0.f;
    /// @brief Buffer Energy (unit: J)
    uint16_t buffer_energy = 0;

    /// @brief Barrel heat of the different Launching Mechanisms \
    /// @brief [0] = 1st 17mm \
    /// @brief [1] = 2nd 17mm \
    /// @brief [2] = 42mm 
    uint16_t barrel_heat = { 0 };

    void initialize_from_data(FrameData& data)
    {

    }

    void print()
    {

    }
};

/// @brief Robot position data. Transmitted at 10Hz to single robot
/// @note ID: 0x0203
struct RobotPosition
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 16;

    /// @brief x/y/z coordinate of robot position (m) \
    /// @brief [0] = X \
    /// @brief [1] = Y \
    /// @brief [2] = Z
    float position[3] = { 0.f };

    /// @brief Direction of this robot’s Speed Monitor Module; unit: degree. True north is 0 degrees.
    /// @note This value is highly dependant on physical attachment of the speed monitor
    float angle = 0.f;

    void initialize_from_data(FrameData& data)
    {

    }

    void print()
    {

    }
};

/// @brief Robot buff data. Transmitted at 3Hz to single robot
/// @note ID: 0x0204
struct RobotBuff
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 1;

    /// @brief Robot's various Buffs \
    /// @brief [0] = Robot’s HP Recovery Buff (in percentage) \
    /// @brief [1] = Robot barrel cooling rate (in absolute value; a value of 5 means a cooling rate of 5 times) \
    /// @brief [2] = Robot defense buff (in percentage) \
    /// @brief [3] = Robot attack buff (in percentage)
    uint8_t buff_status[4] = { 0 };

    void initialize_from_data(FrameData& data)
    {

    }

    void print()
    {

    }
};

/// @brief Air support time data. Transmitted at 10Hz to all our aerial robots
/// @note ID: 0x0205
struct AirSupportTime
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 1;

    /// @brief Status of our aerial robot \
    /// @brief [0] = Aerial robot status: 0 = Cooling, 1 = Done Cooling, 2 = Air support ongoing \
    /// @brief [1] = Time remaining of this status (in seconds) (truncated to lowest integer)
    uint8_t air_status[2] = { 0 };

    void initialize_from_data(FrameData& data)
    {

    }

    void print()
    {

    }
};

/// @brief Damage status data. Transmitted on hurt event to single robot
/// @note ID: 0x0206
struct DamageStatus
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 1;

    /// @brief ID and type of damage \
    /// @brief [0] = ID of the Armor Module hit by projectile. (0 if this is not the reason for damage) \
    /// @brief [1] = Type of HP change: 0 = Projectile, 1 = Module offline, 2 = Shooting too fast, 3 = barrel heat, 4 = Power consumption, 5 = collision
    uint8_t hurt_status[2] = { 0 };

    void initialize_from_data(FrameData& data)
    {

    }

    void print()
    {

    }
};

/// @brief Real-time launching data. Transmitted on projectile launch event to single robot
/// @note ID: 0x0207
struct LaunchingEvent
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 7;

    /// @brief Barrel and projectile data \
    /// @brief [0] = Projectile Type: 1 = 17mm, 2 = 42mm \
    /// @brief [1] = Barrel ID: 1 = 1st 17mm, 2 = 2nd 17mm, 3 = 42mm
    uint8_t barrel_data[2] = { 0 };

    /// @brief Projectile launch speed (unit: Hz)
    uint8_t launching_speed = 0;

    /// @brief Projectile initial speed (unit: m/s)
    float projectile_initial_speed = 0.f;

    void initialize_from_data(FrameData& data)
    {

    }

    void print()
    {

    }
};

/// @brief Projectile Allowance. Transmitted at 10Hz to our hero, standards, sentry, and aerial robots
/// @note ID: 0x0208
struct ProjectileAllowance
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 6;

    /// @brief Projectile allowances (and gold coins) \
    /// @brief [0] = 17mm allowance \
    /// @brief [1] = 42mm allowance \
    /// @brief [2] = remaining gold coins
    uint16_t allowance[3] = { 0 };

    void initialize_from_data(FrameData& data)
    {

    }

    void print()
    {

    }
};

/// @brief Robot RFID status. Transmitted at 3Hz to all our robots (with RFID module)
/// @note ID: 0x0209
struct RFID
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 4;

    /// @brief various data based on whether a robot is occupying a certain point on the map 
    /// @note See Referee System Specification on what each bit represents. Remember it's little endian
    uint32_t RFID_status = 0;

    void initialize_from_data(FrameData& data)
    {

    }

    void print()
    {

    }
};

/// @brief Dart player client command data. Transmitted at 10Hz to all our dart robots
/// @note ID: 0x020A
struct DartCommand
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 6;

    /// @brief Status of the Dart Launching Station \
    /// @brief [0] = door status: 0 = Opened, 1 = Closed, 2 = Opening/Closing \
    /// @brief [1] = dart target: 0 = Outpost, 1 = Base
    uint8_t dart_status[2] = { 0 };

    /// @brief Time stamps on the Dart Launching Station \
    /// @brief [0] = Time remaining in the competition when switching targets (seconds) \
    /// @brief [1] = Time remaining in the competition when the Operator confirms the launch command for the last time (seconds)
    uint16_t time_status[2] = { 0 };

    void initialize_from_data(FrameData& data)
    {

    }

    void print()
    {

    }
};

/// @brief Ground Robot position data. Transmitted at 1Hz to our sentry
/// @note ID: 0x020B
/// @note The intersection of the site perimeter wall near the Red Team’s Official Projectile Supplier is the origin; the orientation of the site’s longer edge facing the Blue Team is the positive x-axis direction, while the orientation of the site’s shorter edge facing the Red Team’s Landing Pad is the positive y-axis direction
struct GroundRobotPosition
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 40;

    /// @brief Position of each ground robots excluding the sentry (meters) \
    /// @brief [0, 1] = X/Y of Hero robot \
    /// @brief [2, 3] = X/Y of Engineer robot \
    /// @brief [4, 5] = X/Y of Standard 3 robot \
    /// @brief [6, 7] = X/Y of Standard 4 robot \
    /// @brief [8, 9] = X/Y of Standard 5 robot
    float positions[10] = { 0.f };

    void initialize_from_data(FrameData& data)
    {

    }

    void print()
    {

    }
};

/// @brief Radar marked progress data. Transmitted at 1Hz to our radar
/// @note ID: 0x020C
struct RadarProgress
{
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 6;

    /// @brief Marked progress of the enemy team's ground robots (0 - 120) \
    /// @brief [0] = Marked progress of Hero \
    /// @brief [1] = Marked progress of Engineer \
    /// @brief [2] = Marked progress of Standard 3\
    /// @brief [3] = Marked progress of Standard 4\
    /// @brief [4] = Marked progress of Standard 5 \
    /// @brief [5] = Marked progress of Sentry 
    uint8_t mark_progress[6] = { 0 };

    void initialize_from_data(FrameData& data)
    {

    }

    void print()
    {

    }
};



#endif // REF_SYSTEM_PACKET_DEFINITIONS_HPP