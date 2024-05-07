#ifndef REF_SYSTEM_PACKET_DEFINITIONS_HPP
#define REF_SYSTEM_PACKET_DEFINITIONS_HPP

#include <Arduino.h>

/// @brief Maximum size of a Ref System packet in bytes
constexpr uint16_t REF_MAX_PACKET_SIZE = 128;
/// @brief Maximum valid command ID for Ref System packets
constexpr uint16_t REF_MAX_COMMAND_ID = 0x0308;

/*--- Ref System Frame Structs ---*/

/// @brief Base enum maping the ref structs to their ID
enum FrameType {
    /// @brief Competition status data
    GAME_STATUS = 0x0001,
    /// @brief Competition result data
    GAME_RESULT = 0x0002,
    /// @brief Robot health data
    GAME_ROBOT_HP = 0x0003,
    /// @brief Site event data
    EVENT_DATA = 0x0101,
    /// @brief Action identifier data of the Official Projectile Supplier
    PROJECTILE_SUPPLIER_STATUS = 0x0102,
    /// @brief Referee warning data
    REFEREE_WARNING = 0x0104,
    /// @brief Dart launching data
    DART_STATUS = 0x0105,
    /// @brief Robot performance system data
    ROBOT_PERFORMANCE = 0x0201,
    /// @brief Real-time chassis power and barrel heat data
    ROBOT_POWER_HEAT = 0x0202,
    /// @brief Robot position data
    ROBOT_POSITION = 0x0203,
    /// @brief Robot buff data
    ROBOT_BUFF = 0x0204,
    /// @brief Air support time data
    AIR_SUPPORT_STATUS = 0x0205,
    /// @brief Damage status data
    DAMAGE_STATUS = 0x0206,
    /// @brief Real-time launching data
    LAUNCHING_STATUS = 0x0207,
    /// @brief Projectile allowance data
    PROJECTILE_ALLOWANCE = 0x0208,
    /// @brief RFID status data
    RFID_STATUS = 0x0209,
    /// @brief Dart command data
    DART_COMMAND = 0x020A,
    /// @brief Ground robot positions data
    GROUND_ROBOT_POSITIONS = 0x020B,
    /// @brief Radar progress data
    RADAR_PROGRESS = 0x020C,
    /// @brief Sentry decision data
    SENTRY_DECISION = 0x020D,
    /// @brief Radar decision data
    RADAR_DECISION = 0x020E,
    /// @brief Robot interaction data
    ROBOT_INTERACTION = 0x0301,
    /// @brief Data about the interaction between the Custom Controller and robots
    CUSTOM_CONTROLLER_ROBOT = 0x0302,
    /// @brief Player client's small map interaction data
    SMALL_MAP_COMMAND = 0x0303,
    /// @brief Keyboard, mouse, and remote control data
    KBM_INTERACTION = 0x0304,
    /// @brief Radar data received by player clients' Small Maps
    SMALL_MAP_RADAR_POSITION = 0x0305,
    /// @brief Data about the interaction between the Custom Controller and player clients
    CUSTOM_CONTROLLER_CLIENT = 0x0306,
    /// @brief Sentry data received by player clients' Small Maps
    SMALL_MAP_SENTRY_COMMAND = 0x0307,
    /// @brief Robot data received by player clients' Small Map
    SMALL_MAP_ROBOT_DATA = 0x0308
};

/// @brief Struct for the Frame header portion
struct FrameHeader {
    /// @brief size of packet sent by Ref System in bytes
    static const uint8_t packet_size = 5;

    /// @brief Start of Frame byte, should be 0xA5 if valid frame
    uint8_t SOF = 0;
    /// @brief length of the FrameData portion of a Frame
    uint16_t data_length = 0;
    /// @brief Sequence number, increments per frame, wraps around 255
    uint8_t sequence = 0;

    /// @brief An 8-bit CRC for the FrameHeader only
    uint8_t CRC = 0;

    /// @brief Prints the FrameHeader
    void print() {
        Serial.printf("\tSOF: %x\n", SOF);
        Serial.printf("\tLength: %u\n", data_length);
        Serial.printf("\tSequence: %u\n", sequence);
        Serial.printf("\tCRC: %x\n", CRC);
    }
};

/// @brief Struct for the Frame data portion
struct FrameData {
    /// @brief Data array to hold the Frame data portion
    uint8_t data[REF_MAX_PACKET_SIZE] = { 0 };

    /// @brief Helpful index operator. Allows array-like indexing from the object itself
    /// @param index index
    /// @return uint8_t data at index
    uint8_t operator[](int index) {
        return data[index];
    }
};

/// @brief Struct for the entire Frame. Consists of a FrameHeader, Command ID, FrameData, and CRC
struct Frame {
    /// @brief Header portion of a Frame
    FrameHeader header {};
    /// @brief Command ID potion of a Frame
    uint16_t commandID = 0;
    /// @brief Data portion of a Frame
    FrameData data {};
    /// @brief 16-bit CRC for the entire Frame
    uint16_t CRC = 0;

    /// @brief Prints the Frame
    void print() {
        Serial.println("Read Frame:");
        header.print();
        Serial.printf("Command ID: %.2x\n", commandID);
        for (int i = 0; i < header.data_length; i++) {
            Serial.printf("%x ", data[i]);
        }
        Serial.println();
        Serial.printf("CRC: %.2x\n", CRC);
    }
};

/*--- Ref System Command ID Packet Structs ---*/

/// @brief Competition status data
/// @note transmitted at a fixed frequency of 1 Hz to all robots
/// @note ID: 0x0001
struct GameStatus {
    /// @brief Size of the GameStatus packet in bytes
    static const uint8_t packet_size = 11;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};
    
    /// @brief Competition type \n
    /// @brief 1: RMUC. 2: RMUT. 3: RMUA. 4: RMUL 3v3. 5: RUML 1v1.
    uint8_t competition_type = 0;
    /// @brief Current stage of the competition \n
    /// @brief 0: pre-competition. 1: preparation. 2: 15s Ref System initialization. 3: 5s countdown. 4: In competition. 5: Result calculation.
    uint8_t current_stage = 0;
    /// @brief Remaining time of the current round in seconds
    uint16_t round_time_remaining = 0;
    /// @brief UNIX time, effective after the robot is correctly connected to the Referee System's NTP server
    uint64_t unix_time = 0;

    /// @brief Prints the GameStatus packet
    void print() {
        Serial.println("GameStatus:");
        Serial.printf("\tCompetition Type: %u\n", competition_type);
        Serial.printf("\tCurrent Stage: %u\n", current_stage);
        Serial.printf("\tRound Time Remaining: %u\n", round_time_remaining);
        Serial.printf("\tUnix Time: %lu\n", unix_time);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        competition_type = data[0] & 0x0F;
        current_stage = (data[0] >> 4) & 0x0F;
        round_time_remaining = (data[2] << 8) | data[1];
        unix_time |= data[10];
        unix_time <<= 8;
        unix_time |= data[9];
        unix_time <<= 8;
        unix_time |= data[8];
        unix_time <<= 8;
        unix_time |= data[7];
        unix_time <<= 8;
        unix_time |= data[6];
        unix_time <<= 8;
        unix_time |= data[5];
        unix_time <<= 8;
        unix_time |= data[4];
        unix_time <<= 8;
        unix_time |= data[3];
    }
};

/// @brief Competition result data
/// @note transmitted at the end of the competition
/// @note ID: 0x0002
struct GameResult {
    /// @brief Size of the GameResult packet in bytes
    static const uint8_t packet_size = 1;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Winner
    /// @brief 0: Draw. 1: Red team wins. 2: Blue team wins.
    uint8_t winner = 0;

    /// @brief Prints the GameResult packet
    void print() {
        Serial.println("GameResult:");
        Serial.printf("\tWinner: %u\n", winner);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        winner = data[0];
    }
};

/// @brief Robot health data
/// @note transmitted at a fixed frequency of 3 Hz to all robots
/// @note ID: 0x0003
struct GameRobotHP {
    /// @brief Size of the GameRobotHP packet in bytes
    static const uint8_t packet_size = 32;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Red team's robot HPs
    /// @brief 0: Robot 1. 1: Robot 2. 2: Robot 3. 3: Robot 4. 4: Robot 5. 5: Robot 7. 6: Outpost. 7: Base.
    uint16_t red_team_HP[8] = { 0 };

    /// @brief Blue team's robot HPs
    /// @brief 0: Robot 1. 1: Robot 2. 2: Robot 3. 3: Robot 4. 4: Robot 5. 5: Robot 7. 6: Outpost. 7: Base.
    uint16_t blue_team_HP[8] = { 0 };

    /// @brief Prints the GameRobotHP packet
    void print() {
        Serial.println("GameRobotHP:");
        Serial.println("Red Team:");
        for (int i = 0; i < 8; i++) {
            Serial.printf("\tRobot %u: %u\n", i + 1, red_team_HP[i]);
        }
        Serial.println("Blue Team:");
        for (int i = 0; i < 8; i++) {
            Serial.printf("\tRobot %u: %u\n", i + 1, blue_team_HP[i]);
        }
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        for (int i = 0; i < 8; i++) {
            red_team_HP[i] = (data[2 * i + 1] << 8) | data[2 * i];
            blue_team_HP[i] = (data[16 + 2 * i + 1] << 8) | data[16 + 2 * i];
        }
    }
};

/// @brief Site event data
/// @note transmitted at a fixed frequency of 1 Hz to all of our robots
/// @note ID: 0x0101
struct EventData {
    /// @brief Size of the EventData packet in bytes
    static const uint8_t packet_size = 4;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Site event type
    /// @todo Fill this out before china
    uint32_t site_event_data = 0;

    /// @brief Prints the EventData packet
    void print() {
        Serial.println("EventData:");
        Serial.printf("\tSite Event Data: %.8x\n", site_event_data);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        site_event_data = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
    }
};

/// @brief Action identifier data of the Official Projectile Supplier
/// @note transmitted when the Official Projectile Supplier releases projectiles to all of our robots
/// @note ID: 0x0102
struct ProjectileSupplierStatus {
    /// @brief Size of the ProjectileSupplierStatus packet in bytes
    static const uint8_t packet_size = 4;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Reserved
    uint8_t reserved = 0;
    /// @brief ID of the reloading robot
    uint8_t reloading_robot_ID = 0;
    /// @brief Status of the supplier
    /// @brief 0: Closed. 1: Preparing. 2: Releasing.
    uint8_t supplier_status = 0;
    /// @brief Number of projectiles supplied. Max 200. Supplied in 50 ball increments.
    uint8_t num_projectiles_supplied = 0;

    /// @brief Prints the ProjectileSupplierStatus packet
    void print() {
        Serial.println("ProjectileSupplierStatus:");
        Serial.printf("\tReloading Robot ID: %u\n", reloading_robot_ID);
        Serial.printf("\tSupplier Status: %u\n", supplier_status);
        Serial.printf("\tNum Projectiles Supplied: %u\n", num_projectiles_supplied);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        reserved = data[0];
        reloading_robot_ID = data[1];
        supplier_status = data[2];
        num_projectiles_supplied = data[3];
    }
};

/// @brief Referee warning data
/// @note transmitted when one's team is issued a penalty/forfeiture and at a fixed frequency of 1 Hz in other cases to all robots of the penalized team
/// @note ID: 0x0104
struct RefereeWarning {
    /// @brief Size of the RefereeWarning packet in bytes
    static const uint8_t packet_size = 3;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Level of penalty that was last received by the own side.
    /// @brief 1: Both teams received yellow card. 2: Yellow card. 3: Red card. 4: Forfeiture.
    uint8_t last_received_severity = 0;
    /// @brief ID of the own side's offending robot that received the last penalty
    /// @note This is in Ref's global robot ID, not the team's robot ID (ie red 1 is ID 1, blue 1 is ID 101)
    uint8_t last_received_robot_ID = 0;
    /// @brief Number of violations (at the corresponding penalty level) triggered by the own side's offending robot that received the last penalty.
    /// @note Default value is 0.
    uint8_t last_num_violations = 0;

    /// @brief Prints the RefereeWarning packet
    void print() {
        Serial.println("RefereeWarning:");
        Serial.printf("\tLast Received Severity: %u\n", last_received_severity);
        Serial.printf("\tLast Received Robot ID: %u\n", last_received_robot_ID);
        Serial.printf("\tLast Num Violations: %u\n", last_num_violations);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        last_received_severity = data[0];
        last_received_robot_ID = data[1];
        last_num_violations = data[2];
    }
};

/// @brief Dart launching data
/// @note transmitted at a fixed frequency of 1 Hz to all of our robots
/// @note ID: 0x0105
struct DartStatus {
    /// @brief Size of the DartStatus packet in bytes
    static const uint8_t packet_size = 3;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Time remaining for the dart to be launched in seconds
    uint8_t time_remaining = 0;
    /// @brief Target that was last hit by a dart of the own side.
    /// @brief 0: Default. 1: Outpost. 2: Fixed target in Base. 3: Random target in Base.
    uint16_t target_last_hit : 2;
    /// @brief Total number of recent hits to a target in the opponent team
    /// @brief 0: Default. 4: Max.
    uint16_t num_recent_hits : 3;
    /// @brief Target currently selected to be hit by the dart.
    /// @brief 0: No target or Outpost. 1: Fixed target in Base. 2: Random target in Base.
    uint16_t current_target : 2;
    /// @brief Reserved.
    uint16_t reserved : 9;

    /// @brief Prints the DartStatus packet
    void print() {
        Serial.println("DartStatus:");
        Serial.printf("\tTime Remaining: %u\n", time_remaining);
        Serial.printf("\tTarget Last Hit: %u\n", target_last_hit);
        Serial.printf("\tNum Recent Hits: %u\n", num_recent_hits);
        Serial.printf("\tCurrent Target: %u\n", current_target);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        time_remaining = data[0];
        target_last_hit = (data[1] & 0x03);
        num_recent_hits = (data[1] >> 2) & 0x07;
        current_target = (data[1] >> 5) & 0x03;
        reserved = (data[2] << 8) | data[3];
    }    
};

/// @brief Robot performance system data
/// @note transmitted at a fixed frequency of 10 Hz to a specific robot
/// @note ID: 0x0201
struct RobotPerformance {
    /// @brief Size of the RobotPerformance packet in bytes
    static const uint8_t packet_size = 13;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief ID of the robot
    uint8_t robot_ID = 0;
    /// @brief Level of the robot
    uint8_t robot_level = 0;
    /// @brief Current HP of the robot
    uint16_t current_HP = 0;
    /// @brief Maximum HP of the robot
    uint16_t max_HP = 0;
    /// @brief Cooling rate of the robot, in units per second
    uint16_t barrel_cooling_rate = 0;
    /// @brief Maximum heating limit of the robot's barrel
    uint16_t barrel_heat_limit = 0;
    /// @brief Chassis power usage limit (unknown units)
    uint16_t chassis_power_limit = 0;
    /// @brief Whether gimbol line is powered
    uint8_t gimbol_power_active : 1;
    /// @brief Whether chassis line is powered
    uint8_t chassis_power_active : 1;
    /// @brief Whether shooter line is powered
    uint8_t shooter_power_active : 1;
    /// @brief Reserved
    uint8_t reserved : 5;

    /// @brief Prints the RobotPerformance packet
    void print() {
        Serial.println("RobotPerformance:");
        Serial.printf("\tRobot ID: %u\n", robot_ID);
        Serial.printf("\tRobot Level: %u\n", robot_level);
        Serial.printf("\tCurrent HP: %u\n", current_HP);
        Serial.printf("\tMax HP: %u\n", max_HP);
        Serial.printf("\tBarrel Cooling Rate: %u\n", barrel_cooling_rate);
        Serial.printf("\tBarrel Heat Limit: %u\n", barrel_heat_limit);
        Serial.printf("\tChassis Power Limit: %u\n", chassis_power_limit);
        Serial.printf("\tGimbol Power Active: %u\n", gimbol_power_active);
        Serial.printf("\tChassis Power Active: %u\n", chassis_power_active);
        Serial.printf("\tShooter Power Active: %u\n", shooter_power_active);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        robot_ID = data[0];
        robot_level = data[1];
        current_HP = (data[3] << 8) | data[2];
        max_HP = (data[5] << 8) | data[4];
        barrel_cooling_rate = (data[7] << 8) | data[6];
        barrel_heat_limit = (data[9] << 8) | data[8];
        chassis_power_limit = (data[11] << 8) | data[10];
        gimbol_power_active = data[12] & 0x01;
        chassis_power_active = (data[12] >> 1) & 0x01;
        shooter_power_active = (data[12] >> 2) & 0x01;
        reserved = (data[12] >> 3) & 0x1F;
    }
};

/// @brief Real-time chassis power and barrel heat data
/// @note transmitted at a fixed frequency of 50 Hz to a specific robot
/// @note ID: 0x0202
struct RobotPowerHeat {
    /// @brief Size of the RobotPower packet in bytes
    static const uint8_t packet_size = 16;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Output voltage of the chassis port in the Power Management Module; unit: mV
    uint16_t chassis_voltage_output = 0;
    /// @brief Output current of the chassis port in the Power Management Module; unit: mA
    uint16_t chassis_current_output = 0;
    /// @brief Chassis power; unit: W
    float chassis_power = 0.f;
    /// @brief Buffer energy; unit: J
    uint16_t buffer_energy = 0;
    /// @brief Barrel heat of the 1st 17mm Launching Mechanism
    uint16_t barrel_heat_1_17mm = 0;
    /// @brief Barrel heat of the 2nd 17mm Launching Mechanism
    uint16_t barrel_heat_2_17mm = 0;
    /// @brief Barrel heat of the 42mm Launching Mechanism
    uint16_t barrel_heat_42mm = 0;

    /// @brief Prints the RobotPowerHeat packet
    void print() {
        Serial.println("RobotPowerHeat:");
        Serial.printf("\tChassis Voltage Output: %u\n", chassis_voltage_output);
        Serial.printf("\tChassis Current Output: %u\n", chassis_current_output);
        Serial.printf("\tChassis Power: %f\n", chassis_power);
        Serial.printf("\tBuffer Energy: %u\n", buffer_energy);
        Serial.printf("\tBarrel Heat 1 17mm: %u\n", barrel_heat_1_17mm);
        Serial.printf("\tBarrel Heat 2 17mm: %u\n", barrel_heat_2_17mm);
        Serial.printf("\tBarrel Heat 42mm: %u\n", barrel_heat_42mm);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        chassis_voltage_output = (data[1] << 8) | data[0];
        chassis_current_output = (data[3] << 8) | data[2];

        uint32_t chassis_power_raw = (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4];
        memcpy(&chassis_power, &chassis_power_raw, sizeof(chassis_power));
        
        buffer_energy = (data[9] << 8) | data[8];
        barrel_heat_1_17mm = (data[11] << 8) | data[10];
        barrel_heat_2_17mm = (data[13] << 8) | data[12];
        barrel_heat_42mm = (data[15] << 8) | data[14];
    }
};

/// @brief Robot position data
/// @note transmitted at a fixed frequency of 1 Hz to a specific robot
/// @note ID: 0x0203
struct RobotPosition {
    /// @brief Size of the RobotPosition packet in bytes
    static const uint8_t packet_size = 16;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief The x-coordinate of the robot's position; unit: m.
    float x = 0.f;
    /// @brief The y-coordinate of the robot's position; unit: m.
    float y = 0.f;
    /// @brief Direction of the robot's Speed Monitor Module; unit: degree. True north is 0 degrees.
    float angle = 0.f;

    /// @brief Prints the RobotPosition packet
    void print() {
        Serial.println("RobotPosition:");
        Serial.printf("\tX: %f\n", x);
        Serial.printf("\tY: %f\n", y);
        Serial.printf("\tAngle: %f\n", angle);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);

        uint32_t x_raw = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
        memcpy(&x, &x_raw, sizeof(x));
        uint32_t y_raw = (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4];
        memcpy(&y, &y_raw, sizeof(y));
        uint32_t angle_raw = (data[11] << 24) | (data[10] << 16) | (data[9] << 8) | data[8];
        memcpy(&angle, &angle_raw, sizeof(angle));
    }
};

/// @brief Robot buff data
/// @note transmitted at a fixed frequency of 3 Hz to a specific robot
/// @note ID: 0x0204
struct RobotBuff {
    /// @brief Size of the RobotBuff packet in bytes
    static const uint8_t packet_size = 6;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Robot's HP recovery buff (in percentage; a value of 10 indicates that HP recovery per second is 10% of the maximum HP.)
    uint8_t hp_recovery = 0;
    /// @brief Robot's barrel cooling rate (in absolute value; a value of 5 indicates a cooling rate of 5 times.)
    uint8_t heat_cooling = 0;
    /// @brief Robot's defense buff (in percentage; a value of 50 indicates a defense buff of 50%.)
    uint8_t defence = 0;
    /// @brief Robot's negative defense buff (in percentage; a value of 30 indicates a defense buff of -30%.)
    uint8_t negative_defence = 0;
    /// @brief Robot's attack buff (in percentage; a value of 50 indicates an attack buff of 50%.)
    uint16_t attack = 0;

    /// @brief Prints the RobotBuff packet
    void print() {
        Serial.println("RobotBuff:");
        Serial.printf("\tHP Recovery: %u\n", hp_recovery);
        Serial.printf("\tHeat Cooling: %u\n", heat_cooling);
        Serial.printf("\tDefence: %u\n", defence);
        Serial.printf("\tNegative Defence: %u\n", negative_defence);
        Serial.printf("\tAttack: %u\n", attack);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        hp_recovery = data[0];
        heat_cooling = data[1];
        defence = data[2];
        negative_defence = data[3];
        attack = (data[5] << 8) | data[4];
    }
};

/// @brief Air support time data
/// @note transmitted at a fixed frequency of 1 Hz to our aerial robots
/// @note ID: 0x0205
struct AirSupportStatus {
    /// @brief Size of the AirSupportStatus packet in bytes
    static const uint8_t packet_size = 2;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Aerial Robot's status
    /// @brief 0: Cooling. 1: Cooling finished. 2: Active.
    uint8_t status = 0;
    /// @brief Remainint time of the current status in seconds
    /// @note Rounded down to the nearest integer by Ref.
    uint8_t time_remaining = 0;

    /// @brief Prints the AirSupportStatus packet
    void print() {
        Serial.println("AirSupportStatus:");
        Serial.printf("\tStatus: %u\n", status);
        Serial.printf("\tTime Remaining: %u\n", time_remaining);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        status = data[0];
        time_remaining = data[1];
    }
};

/// @brief Damage status data
/// @note transmitted after the damage occurs to a specific robot
/// @note ID: 0x0206
struct DamageStatus {
    /// @brief Size of the DamageStatus packet in bytes
    static const uint8_t packet_size = 1;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief ID of the armor plate that was hit
    /// @note This is only valid if the damage was from projectiles, collisions, going offline, or Speed Monitor Module going offline.
    uint8_t armor_plate_ID : 4;
    /// @brief Type of damage
    /// @brief 0: Projectiles. 1: Critical Ref System Module goes offline. 2: Shooting too fast (speed). 3: Barrel overheat. 4: Power limit exceeded. 5: Collision.
    uint8_t damage_type : 4;

    /// @brief Prints the DamageStatus packet
    void print() {
        Serial.println("DamageStatus:");
        Serial.printf("\tArmor Plate ID: %u\n", armor_plate_ID);
        Serial.printf("\tDamage Type: %u\n", damage_type);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        armor_plate_ID = data[0] & 0x0F;
        damage_type = (data[0] >> 4) & 0x0F;
    }
};

/// @brief Real-time launching data
/// @note transmitted after a projectile is launched to a specific robot
/// @note ID: 0x0207
struct LaunchingStatus {
    /// @brief Size of the LaunchingStatus packet in bytes
    static const uint8_t packet_size = 7;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Type of the projectile
    /// @brief 1: 17mm. 2: 42mm.
    uint8_t projectile_type = 0;
    /// @brief The ID of which launching mechanism was used
    /// @brief 1: 1st 17mm. 2: 2nd 17mm. 3: 42mm.
    uint8_t launching_mechanism = 0;
    /// @brief The frequency of firing; unit: Hz
    uint8_t launching_frequency = 0;
    /// @brief The initial speed of the projectile; unit: m/s
    float initial_speed = 0.f;

    /// @brief Prints the LaunchingStatus packet
    void print() {
        Serial.println("LaunchingStatus:");
        Serial.printf("\tProjectile Type: %u\n", projectile_type);
        Serial.printf("\tLaunching Mechanism: %u\n", launching_mechanism);
        Serial.printf("\tLaunching Frequency: %u\n", launching_frequency);
        Serial.printf("\tInitial Speed: %f\n", initial_speed);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        projectile_type = data[0];
        launching_mechanism = data[1];
        launching_frequency = data[2];
        uint32_t initial_speed_raw = (data[6] << 24) | (data[5] << 16) | (data[4] << 8) | data[3];
        memcpy(&initial_speed, &initial_speed_raw, sizeof(initial_speed));
    }
};

/// @brief Projectile allowance data
/// @note transmitted at a fixed frequency of 10 Hz to a specific robot
/// @note ID: 0x0208
struct ProjectileAllowance {
    /// @brief Size of the ProjectileAllowance packet in bytes
    static const uint8_t packet_size = 6;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Number of 17mm projectiles remaining
    uint16_t num_17mm = 0;
    /// @brief Number of 42mm projectiles remaining
    uint16_t num_42mm = 0;
    /// @brief Num Gold Coins remaining
    uint16_t num_gold = 0;

    /// @brief Prints the ProjectileAllowance packet
    void print() {
        Serial.println("ProjectileAllowance:");
        Serial.printf("\tNum 17mm: %u\n", num_17mm);
        Serial.printf("\tNum 42mm: %u\n", num_42mm);
        Serial.printf("\tNum Gold: %u\n", num_gold);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        num_17mm = (data[1] << 8) | data[0];
        num_42mm = (data[3] << 8) | data[2];
        num_gold = (data[5] << 8) | data[4];
    }
};

/// @brief Robot RFID module status
/// @note transmitted at a fixed frequency of 3 Hz to a our robots
/// @note ID: 0x0209
struct RFIDStatus {
    /// @brief Size of the RFIDStatus packet in bytes
    static const uint8_t packet_size = 4;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    // Meaning of bit value 0 or 1: whether the Buff Point's RFID card is detected.

    /// @brief own side's Base Buff Point
    uint32_t our_base_buff_point : 1;
    /// @brief own side's Ring-Shaped Elevated Ground Buff Point
    uint32_t our_ring_buff_point : 1;
    /// @brief opponent's Ring-Shaped Elevated Ground Buff Point
    uint32_t their_ring_buff_point : 1;
    /// @brief own side's R3/B3 Trapezoid-Shaped Elevated Ground Buff Point
    uint32_t our_3_trapezoid_buff_point : 1;
    /// @brief opponent's R3/B3 Trapezoid-Shaped Elevated Ground Buff Point
    uint32_t their_3_trapezoid_buff_point : 1;
    /// @brief own side's R4/B4 Trapezoid-Shaped Elevated Ground Buff Point
    uint32_t our_4_trapezoid_buff_point : 1;
    /// @brief opponent's R4/B4 Trapezoid-Shaped Elevated Ground Buff Point
    uint32_t their_4_trapezoid_buff_point : 1;
    /// @brief own side's Power Rune Activation Point
    uint32_t our_power_rune_point : 1;
    /// @brief own side's Launch Ramp Buff Point (in front of the Launch Ramp near own side)
    uint32_t our_launch_ramp_buff_point_front : 1;
    /// @brief own side's Launch Ramp Buff Point (behind the Launch Ramp near own side)
    uint32_t our_launch_ramp_buff_point_back : 1;
    /// @brief opponent's Launch Ramp Buff Point (in front of the Launch Ramp near the other side)
    uint32_t their_launch_ramp_buff_point_front : 1;
    /// @brief opponent's Launch Ramp Buff Point (behind the Launch Ramp near the other side)
    uint32_t their_launch_ramp_buff_point_back : 1;
    /// @brief own side's Outpost Buff Point
    uint32_t our_outpost_buff_point : 1;
    /// @brief own side's Restoration Zone (deemed activated if anyone is detected)
    uint32_t our_restoration_zone : 1;
    /// @brief own side's Sentry Patrol Zones
    uint32_t our_sentry_patrol_zones : 1;
    /// @brief opponent's Sentry Patrol Zones
    uint32_t their_sentry_patrol_zones : 1;
    /// @brief own side's Large Resource Island Buff Point
    uint32_t our_large_resource_buff_point : 1;
    /// @brief opponent's Large Resource Island Buff Point
    uint32_t their_large_resource_buff_point : 1;
    /// @brief own side's Exchange Zone
    uint32_t our_exchange_zone : 1;
    /// @brief Central Buff Point (for RMUL only)
    uint32_t central_buff_point : 1;
    /// @brief Reserved
    uint32_t reserved : 12;

    /// @brief Prints the RFIDStatus packet
    /// @todo Implement this before china
    void print() {}

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        our_base_buff_point = data[0] & 0x01;
        our_ring_buff_point = (data[0] >> 1) & 0x01;
        their_ring_buff_point = (data[0] >> 2) & 0x01;
        our_3_trapezoid_buff_point = (data[0] >> 3) & 0x01;
        their_3_trapezoid_buff_point = (data[0] >> 4) & 0x01;
        our_4_trapezoid_buff_point = (data[0] >> 5) & 0x01;
        their_4_trapezoid_buff_point = (data[0] >> 6) & 0x01;
        our_power_rune_point = (data[0] >> 7) & 0x01;
        our_launch_ramp_buff_point_front = data[1] & 0x01;
        our_launch_ramp_buff_point_back = (data[1] >> 1) & 0x01;
        their_launch_ramp_buff_point_front = (data[1] >> 2) & 0x01;
        their_launch_ramp_buff_point_back = (data[1] >> 3) & 0x01;
        our_outpost_buff_point = (data[1] >> 4) & 0x01;
        our_restoration_zone = (data[1] >> 5) & 0x01;
        our_sentry_patrol_zones = (data[1] >> 6) & 0x01;
        their_sentry_patrol_zones = (data[1] >> 7) & 0x01;
        our_large_resource_buff_point = data[2] & 0x01;
        their_large_resource_buff_point = (data[2] >> 1) & 0x01;
        our_exchange_zone = (data[2] >> 2) & 0x01;
        central_buff_point = (data[2] >> 3) & 0x01;
        reserved = (data[2] >> 4) & 0x0FFF;
    }
};

/// @brief Dart player's client command data
/// @note transmitted at a fixed frequency of 3 Hz to the Dart Player
/// @note ID: 0x020A
struct DartCommand {
    /// @brief Size of the DartCommand packet in bytes
    static const uint8_t packet_size = 6;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Status of the Launching Status
    /// @brief 0: Opened. 1: Closed. 2: Opening/Closing
    uint8_t status = 0;
    /// @brief Reserved
    uint8_t reserved = 0;
    /// @brief Remaining competition time when the attack target is changed. Unit: s. If no target change occurs, the value is 0 by default
    uint16_t time_remaining_on_target_change = 0;
    /// @brief Remaining competition time when the Operator confirms the launch command for the last time. Unit: s. Initial value: 0.
    uint16_t time_remaining_on_launch_confirm = 0;

    /// @brief Prints the DartCommand packet
    void print() {
        Serial.println("DartCommand:");
        Serial.printf("\tStatus: %u\n", status);
        Serial.printf("\tReserved: %u\n", reserved);
        Serial.printf("\tTime Remaining on Target Change: %u\n", time_remaining_on_target_change);
        Serial.printf("\tTime Remaining on Launch Confirm: %u\n", time_remaining_on_launch_confirm);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        status = data[0];
        reserved = data[1];
        time_remaining_on_target_change = (data[3] << 8) | data[2];
        time_remaining_on_launch_confirm = (data[5] << 8) | data[4];
    }
};

/// @brief Ground Robot position data
/// @note transmitted at a fixed frequency of 1 Hz to our Sentry
/// @note ID: 0x020B
struct GroundRobotPositions {
    /// @brief Size of the RobotPosition packet in bytes
    static const uint8_t packet_size = 40;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief The x-axis coordinate of the own side's Hero Robot; unit: m.
    float hero_x = 0.f;
    /// @brief The y-axis coordinate of the own side's Hero Robot; unit: m.
    float hero_y = 0.f;
    /// @brief The x-axis coordinate of the own side's Engineer Robot; unit: m.
    float engineer_x = 0.f;
    /// @brief The y-axis coordinate of the own side's Engineer Robot; unit: m.
    float engineer_y = 0.f;
    /// @brief The x-axis coordinate of the own side's Standard Robot No. 3; unit: m.
    float standard_3_x = 0.f;
    /// @brief The y-axis coordinate of the own side's Standard Robot No. 3; unit: m.
    float standard_3_y = 0.f;
    /// @brief The x-axis coordinate of the own side's Standard Robot No. 4; unit: m.
    float standard_4_x = 0.f;
    /// @brief The y-axis coordinate of the own side's Standard Robot No. 4; unit: m.
    float standard_4_y = 0.f;
    /// @brief The x-axis coordinate of the own side's Standard Robot No. 5; unit: m.
    float standard_5_x = 0.f;
    /// @brief The y-axis coordinate of the own side's Standard Robot No. 5; unit: m.
    float standard_5_y = 0.f;

    /// @brief Prints the RobotPosition packet
    void print() {
        Serial.println("RobotPosition:");
        Serial.printf("\tHero X: %f\n", hero_x);
        Serial.printf("\tHero Y: %f\n", hero_y);
        Serial.printf("\tEngineer X: %f\n", engineer_x);
        Serial.printf("\tEngineer Y: %f\n", engineer_y);
        Serial.printf("\tStandard 3 X: %f\n", standard_3_x);
        Serial.printf("\tStandard 3 Y: %f\n", standard_3_y);
        Serial.printf("\tStandard 4 X: %f\n", standard_4_x);
        Serial.printf("\tStandard 4 Y: %f\n", standard_4_y);
        Serial.printf("\tStandard 5 X: %f\n", standard_5_x);
        Serial.printf("\tStandard 5 Y: %f\n", standard_5_y);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);

        uint32_t hero_x_raw = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
        memcpy(&hero_x, &hero_x_raw, sizeof(hero_x));
        uint32_t hero_y_raw = (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4];
        memcpy(&hero_y, &hero_y_raw, sizeof(hero_y));
        uint32_t engineer_x_raw = (data[11] << 24) | (data[10] << 16) | (data[9] << 8) | data[8];
        memcpy(&engineer_x, &engineer_x_raw, sizeof(engineer_x));
        uint32_t engineer_y_raw = (data[15] << 24) | (data[14] << 16) | (data[13] << 8) | data[12];
        memcpy(&engineer_y, &engineer_y_raw, sizeof(engineer_y));
        uint32_t standard_3_x_raw = (data[19] << 24) | (data[18] << 16) | (data[17] << 8) | data[16];
        memcpy(&standard_3_x, &standard_3_x_raw, sizeof(standard_3_x));
        uint32_t standard_3_y_raw = (data[23] << 24) | (data[22] << 16) | (data[21] << 8) | data[20];
        memcpy(&standard_3_y, &standard_3_y_raw, sizeof(standard_3_y));
        uint32_t standard_4_x_raw = (data[27] << 24) | (data[26] << 16) | (data[25] << 8) | data[24];
        memcpy(&standard_4_x, &standard_4_x_raw, sizeof(standard_4_x));
        uint32_t standard_4_y_raw = (data[31] << 24) | (data[30] << 16) | (data[29] << 8) | data[28];
        memcpy(&standard_4_y, &standard_4_y_raw, sizeof(standard_4_y));
        uint32_t standard_5_x_raw = (data[35] << 24) | (data[34] << 16) | (data[33] << 8) | data[32];
        memcpy(&standard_5_x, &standard_5_x_raw, sizeof(standard_5_x));
        uint32_t standard_5_y_raw = (data[39] << 24) | (data[38] << 16) | (data[37] << 8) | data[36];
        memcpy(&standard_5_y, &standard_5_y_raw, sizeof(standard_5_y));
    }    
};

/// @brief Radar-marked progress data
/// @note transmitted at a fixed frequency of 1 Hz to our Radar
/// @note ID: 0x020C
struct RadarProgress {
    /// @brief Size of the RadarProgress packet in bytes
    static const uint8_t packet_size = 6;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Marked progress of the opponent's Hero Robot. 0-120
    uint8_t hero = 0;
    /// @brief Marked progress of the opponent's Engineer Robot. 0-120
    uint8_t engineer = 0;
    /// @brief Marked progress of the opponent's Standard Robot No. 3. 0-120
    uint8_t standard_3 = 0;
    /// @brief Marked progress of the opponent's Standard Robot No. 4. 0-120
    uint8_t standard_4 = 0;
    /// @brief Marked progress of the opponent's Standard Robot No. 5. 0-120
    uint8_t standard_5 = 0;
    /// @brief Marked progress of the opponent's Sentry Robot. 0-120
    uint8_t sentry = 0;

    /// @brief Prints the RadarProgress packet
    void print() {
        Serial.println("RadarProgress:");
        Serial.printf("\tHero: %u\n", hero);
        Serial.printf("\tEngineer: %u\n", engineer);
        Serial.printf("\tStandard 3: %u\n", standard_3);
        Serial.printf("\tStandard 4: %u\n", standard_4);
        Serial.printf("\tStandard 5: %u\n", standard_5);
        Serial.printf("\tSentry: %u\n", sentry);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        hero = data[0];
        engineer = data[1];
        standard_3 = data[2];
        standard_4 = data[3];
        standard_5 = data[4];
        sentry = data[5];
    }
};

/// @brief Decision-making data of Sentry Robot
/// @note transmitted at a fixed frequency of 1 Hz to our Sentry
/// @note ID: 0x020D
struct SentryDecision {
    /// @brief Size of the SentryDecision packet in bytes
    static const uint8_t packet_size = 4;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @todo implement before china
    uint32_t sentry_info = 0;

    /// @brief Prints the SentryDecision packet
    void print() {
        Serial.println("SentryDecision:");
        Serial.printf("\tSentry Info: %u\n", sentry_info);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        sentry_info = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
    }
};

/// @brief Decision-making data of Radar
/// @note transmitted at a fixed frequency of 1 Hz to our Radar
/// @note ID: 0x020E
struct RadarDecision {
    /// @brief Size of the RadarDecision packet in bytes
    static const uint8_t packet_size = 1;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @todo implement before china
    uint8_t radar_info = 0;

    /// @brief Prints the RadarDecision packet
    void print() {
        Serial.println("RadarDecision:");
        Serial.printf("\tRadar Info: %u\n", radar_info);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        radar_info = data[0];
    }
};

/// @brief Robot interaction data
/// @note transmitted at a maximum frequency of 10 Hz when triggered by the sender
/// @note ID: 0x0301
struct RobotInteraction {
    /// @brief Size of the RobotInteraction packet in bytes
    static const uint8_t packet_size = 128;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief ID that is specified by user. Not critical to REF
    uint16_t content_id = 0;
    /// @brief ID of the robot that should send this packet
    uint16_t sender_id = 0;
    /// @brief ID of the robot that should receive this packet
    uint16_t receiver_id = 0;

    /// @brief Size (in bytes) of the data array
    uint8_t size = 0;
    /// @brief Actual data array holding our byte reperesentation of whatever were sending
    uint8_t data[REF_MAX_PACKET_SIZE] = { 0 };

    /// @brief Prints the RobotInteraction packet
    void print() {
        for (int i = 0; i < size; i++) {
            Serial.printf("%x ", data[i]);
        }
        Serial.println();
    }

    /// @brief Fills in this struct with the data from a Frame object
    /// @param frame Frame object to extract data from
    void set_data(Frame& frame) {
        size = frame.header.data_length - 6;

        content_id = (frame.data.data[1] << 8) | frame.data.data[0];
        sender_id = (frame.data.data[3] << 8) | frame.data.data[2];
        receiver_id = (frame.data.data[5] << 8) | frame.data.data[4];

        for (int i = 0; i < size; i++) {
            data[i] = frame.data.data[i + 6];
        }
    }
};

/// @brief Data about the interaction between the Custom Controller and robots
/// @note transmitted at a maximum frequency of 30 Hz when triggered by the sender to robots with a VTM link
/// @note ID: 0x0302
struct CustomControllerRobot {
    /// @brief Size of the ControllerRobots packet in bytes
    static const uint8_t packet_size = 30;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Custom data
    uint8_t data[30] = { 0 };

    /// @brief Prints the ControllerRobots packet
    void print() {
        Serial.println("ControllerRobots:");
        for (uint8_t i = 0; i < 30; i++) {
            Serial.printf("\tData[%u]: %u\n", i, data[i]);
        }
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        for (uint8_t i = 0; i < 30; i++) {
            this->data[i] = data[i];
        }
    }
};

/// @brief Player client's small map interaction data
/// @note Transmitted when triggered by the player client to a specific robot
/// @note ID: 0x0303
struct SmallMapCommand {
    /// @brief Size of the SmallMapCommand packet in bytes
    static const uint8_t packet_size = 15;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief The x-axis coordinate of the target position; unit: m.
    /// @note When the target robot ID is sent, the value is 0.
    float target_position_x = 0.f;
    /// @brief The y-axis coordinate of the target position; unit: m.
    /// @note When the target robot ID is sent, the value is 0.
    float target_position_y = 0.f;
    /// @brief The generic key value of the key pressed by the Aerial Gimbal Operator.
    /// @note When no key is pressed, the value is 0.
    uint8_t cmd_keyboard = 0;
    /// @brief The opponent's robot ID.
    /// @note When coordinate data is sent, the value is 0
    uint8_t target_robot_id = 0;
    /// @brief The information source ID.
    uint8_t cmd_source = 0;

    /// @brief Prints the SmallMapCommand packet
    void print() {
        Serial.println("SmallMapCommand:");
        Serial.printf("\tTarget Position X: %f\n", target_position_x);
        Serial.printf("\tTarget Position Y: %f\n", target_position_y);
        Serial.printf("\tCMD Keyboard: %u\n", cmd_keyboard);
        Serial.printf("\tTarget Robot ID: %u\n", target_robot_id);
        Serial.printf("\tCMD Source: %u\n", cmd_source);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        uint32_t target_position_x_raw = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
        memcpy(&target_position_x, &target_position_x_raw, sizeof(target_position_x));
        uint32_t target_position_y_raw = (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4];
        memcpy(&target_position_y, &target_position_y_raw, sizeof(target_position_y));
        cmd_keyboard = data[8];
        target_robot_id = data[9];
        cmd_source = data[10];
    }    
};

/// @brief Keyboard, mouse, and remote control data
/// @note transmitted at a fixed frequency of 30 Hz to a robot with a VTM link
/// @note ID: 0x0304
struct KBMInteraction {
    /// @brief Size of the KBMInteraction packet in bytes
    static const uint8_t packet_size = 12;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief x-axis moving speed of the mouse. A negative value indicates a left movement.
    int16_t mouse_speed_x = 0;
    /// @brief y-axis moving speed of the mouse. A negative value indicates an downward movement.
    int16_t mouse_speed_y = 0;
    /// @brief Scroll wheel's moving speed of the mouse. A negative value indicates a backward movement.
    int16_t scroll_speed = 0;
    /// @brief Whether the mouse's left button is pressed. A value of 0 indicates that it is not pressed, and a value of 1 indicates that it is pressed.
    uint8_t button_left = 0;
    /// @brief Whether the mouse's right button is pressed. A value of 0 indicates that it is not pressed, and a value of 1 indicates that it is pressed
    uint8_t button_right = 0;
    /// @brief Whether the keyboard's W key is pressed.
    uint16_t key_w : 1;
    /// @brief Whether the keyboard's S key is pressed.
    uint16_t key_s : 1;
    /// @brief Whether the keyboard's A key is pressed.
    uint16_t key_a : 1;
    /// @brief Whether the keyboard's D key is pressed.
    uint16_t key_d : 1;
    /// @brief Whether the keyboard's Shift key is pressed.
    uint16_t key_shift : 1;
    /// @brief Whether the keyboard's Ctrl key is pressed.
    uint16_t key_ctrl : 1;
    /// @brief Whether the keyboard's Q key is pressed.
    uint16_t key_q : 1;
    /// @brief Whether the keyboard's E key is pressed.
    uint16_t key_e : 1;
    /// @brief Whether the keyboard's R key is pressed.
    uint16_t key_r : 1;
    /// @brief Whether the keyboard's F key is pressed.
    uint16_t key_f : 1;
    /// @brief Whether the keyboard's G key is pressed.
    uint16_t key_g : 1;
    /// @brief Whether the keyboard's Z key is pressed.
    uint16_t key_z : 1;
    /// @brief Whether the keyboard's X key is pressed.
    uint16_t key_x : 1;
    /// @brief Whether the keyboard's C key is pressed.
    uint16_t key_c : 1;
    /// @brief Whether the keyboard's V key is pressed.
    uint16_t key_v : 1;
    /// @brief Whether the keyboard's B key is pressed.
    uint16_t key_b : 1;
    /// @brief Reserved.
    uint16_t reserved = 0;

    /// @brief Prints the KBMInteraction packet
    void print() {
        Serial.println("KBMInteraction:");
        Serial.printf("\tMouse Speed X: %d\n", mouse_speed_x);
        Serial.printf("\tMouse Speed Y: %d\n", mouse_speed_y);
        Serial.printf("\tScroll Speed: %d\n", scroll_speed);
        Serial.printf("\tButton Left: %u\n", button_left);
        Serial.printf("\tButton Right: %u\n", button_right);
        Serial.printf("\tKey W: %u\n", key_w);
        Serial.printf("\tKey S: %u\n", key_s);
        Serial.printf("\tKey A: %u\n", key_a);
        Serial.printf("\tKey D: %u\n", key_d);
        Serial.printf("\tKey Shift: %u\n", key_shift);
        Serial.printf("\tKey Ctrl: %u\n", key_ctrl);
        Serial.printf("\tKey Q: %u\n", key_q);
        Serial.printf("\tKey E: %u\n", key_e);
        Serial.printf("\tKey R: %u\n", key_r);
        Serial.printf("\tKey F: %u\n", key_f);
        Serial.printf("\tKey G: %u\n", key_g);
        Serial.printf("\tKey Z: %u\n", key_z);
        Serial.printf("\tKey X: %u\n", key_x);
        Serial.printf("\tKey C: %u\n", key_c);
        Serial.printf("\tKey V: %u\n", key_v);
        Serial.printf("\tKey B: %u\n", key_b);
        Serial.printf("\tReserved: %u\n", reserved);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        mouse_speed_x = (data[1] << 8) | data[0];
        mouse_speed_y = (data[3] << 8) | data[2];
        scroll_speed = (data[5] << 8) | data[4];
        button_left = data[6];
        button_right = data[7];
        key_w = data[8] & 0x01;
        key_s = (data[8] >> 1) & 0x01;
        key_a = (data[8] >> 2) & 0x01;
        key_d = (data[8] >> 3) & 0x01;
        key_shift = (data[8] >> 4) & 0x01;
        key_ctrl = (data[8] >> 5) & 0x01;
        key_q = (data[8] >> 6) & 0x01;
        key_e = (data[8] >> 7) & 0x01;
        key_r = data[9] & 0x01;
        key_f = (data[9] >> 1) & 0x01;
        key_g = (data[9] >> 2) & 0x01;
        key_z = (data[9] >> 3) & 0x01;
        key_x = (data[9] >> 4) & 0x01;
        key_c = (data[9] >> 5) & 0x01;
        key_v = (data[9] >> 6) & 0x01;
        key_b = (data[9] >> 7) & 0x01;
        reserved = (data[11] << 8) | data[10];
    }
};

/// @brief Radar data received by player clients' Small Maps
/// @note transmitted at a maximum frequency of 10 Hz to all of our player clients
/// @note ID: 0x0305
struct SmallMapRadarPosition {
    /// @brief Size of the SmallMapRadarPosition packet in bytes
    static const uint8_t packet_size = 10;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief The target robot's ID.
    uint16_t target_ID = 0;
    /// @brief The x-axis coordinate of the target robot; unit: m.
    float target_x = 0.f;
    /// @brief The y-axis coordinate of the target robot; unit: m.
    float target_y = 0.f;

    /// @brief Prints the SmallMapRadarPosition packet
    void print() {
        Serial.println("SmallMapRadarPosition:");
        Serial.printf("\tTarget ID: %u\n", target_ID);
        Serial.printf("\tTarget X: %f\n", target_x);
        Serial.printf("\tTarget Y: %f\n", target_y);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        target_ID = (data[1] << 8) | data[0];
        uint32_t target_x_raw = (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2];
        memcpy(&target_x, &target_x_raw, sizeof(target_x));
        uint32_t target_y_raw = (data[9] << 24) | (data[8] << 16) | (data[7] << 8) | data[6];
        memcpy(&target_y, &target_y_raw, sizeof(target_y));
    }
};

/// @brief Data about the interaction between the Custom Controller and player clients
/// @note transmitted at a maximum frequency of 30 Hz when triggered by the sender to player client
/// @note ID: 0x0306
struct CustomControllerClient {
    /// @brief Size of the ControllerClient packet in bytes
    static const uint8_t packet_size = 8;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief value of Key 1
    uint8_t key_1 = 0;
    /// @brief value of Key 2
    uint8_t key_2 = 0;
    /// @brief mouse's x-axis pixel position
    /// @note Origin is top left corner
    uint16_t mouse_x : 12;
    /// @brief mouse's left button
    uint16_t mouse_left : 4;
    /// @brief mouse's y-axis pixel position
    /// @note Origin is top left corner
    uint16_t mouse_y : 12;
    /// @brief mouse's right button
    uint16_t mouse_right : 4;
    /// @brief Reserved
    uint16_t reserved = 0;
    
    /// @brief Prints the ControllerClient packet
    void print() {
        Serial.println("ControllerClient:");
        Serial.printf("\tKey 1: %u\n", key_1);
        Serial.printf("\tKey 2: %u\n", key_2);
        Serial.printf("\tMouse X: %u\n", mouse_x);
        Serial.printf("\tMouse Left: %u\n", mouse_left);
        Serial.printf("\tMouse Y: %u\n", mouse_y);
        Serial.printf("\tMouse Right: %u\n", mouse_right);
        Serial.printf("\tReserved: %u\n", reserved);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        key_1 = data[0];
        key_2 = data[1];
        mouse_x = (data[3] << 8) | data[2];
        mouse_left = data[4] & 0x0F;
        mouse_y = (data[6] << 8) | data[5];
        mouse_right = (data[4] >> 4) & 0x0F;
        reserved = (data[8] << 8) | data[7];
    }
};

/// @brief Sentry data received by player clients' Small Maps
/// @note transmitted at a maximum frequency of 1 Hz to a specific player client
/// @note ID: 0x0307
struct SmallMapSentryCommand {
    /// @brief Size of the SmallMapSentryPosition packet in bytes
    static const uint8_t packet_size = 103;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Specific command sent to the sentry
    /// @brief 1: Go to target point to attack. 2: Go to target point to defend. 3: Go to target point;
    uint8_t command = 0;
    /// @brief The x-axis coordinate of the target position; unit: dm.
    uint16_t start_x = 0;
    /// @brief The y-axis coordinate of the target position; unit: dm.
    uint16_t start_y = 0;
    /// @brief The array of x-axis incremental values of route points; unit: dm.
    int8_t delta_x[49] = {0};
    /// @brief The array of y-axis incremental values of route points; unit: dm.
    int8_t delta_y[49] = {0};
    /// @brief The ID of the sender.
    uint16_t sender_ID = 0;

    /// @brief Prints the SmallMapSentryPosition packet
    void print() {
        Serial.println("SmallMapSentryPosition:");
        Serial.printf("\tCommand: %u\n", command);
        Serial.printf("\tStart X: %u\n", start_x);
        Serial.printf("\tStart Y: %u\n", start_y);
        Serial.println("\tDelta X:");
        for (uint8_t i = 0; i < 49; i++) {
            Serial.printf("\t\t%d\n", delta_x[i]);
        }
        Serial.println("\tDelta Y:");
        for (uint8_t i = 0; i < 49; i++) {
            Serial.printf("\t\t%d\n", delta_y[i]);
        }
        Serial.printf("\tSender ID: %u\n", sender_ID);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        command = data[0];
        start_x = (data[2] << 8) | data[1];
        start_y = (data[4] << 8) | data[3];
        for (uint8_t i = 0; i < 49; i++) {
            delta_x[i] = data[5 + i];
        }
        for (uint8_t i = 0; i < 49; i++) {
            delta_y[i] = data[54 + i];
        }
        sender_ID = (data[106] << 8) | data[105];
    }
};

/// @brief Robot data received by player clients' Small Map
/// @note transmitted at a maximum frequency of 3 Hz to all of our player clients
/// @note ID: 0x0308
struct SmallMapRobotData {
    /// @brief Size of the SmallMapRobotPosition packet in bytes
    static const uint8_t packet_size = 34;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Sender ID
    uint16_t sender_ID = 0;
    /// @brief Receiver ID
    uint16_t receiver_ID = 0;
    /// @brief Character array
    uint8_t data[30] = {0};

    /// @brief Prints the SmallMapRobotPosition packet
    void print() {
        Serial.println("SmallMapRobotPosition:");
        Serial.printf("\tSender ID: %u\n", sender_ID);
        Serial.printf("\tReceiver ID: %u\n", receiver_ID);
        Serial.println("\tData:");
        for (uint8_t i = 0; i < 30; i++) {
            Serial.printf("\t\t%u\n", data[i]);
        }
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        sender_ID = (data[1] << 8) | data[0];
        receiver_ID = (data[3] << 8) | data[2];
        for (uint8_t i = 0; i < 30; i++) {
            this->data[i] = data[4 + i];
        }
    }
};

/// @brief Encompassing all read-able packet structs of the Ref System
struct RefData {
    /// @brief Competition status data
    GameStatus game_status{};
    /// @brief Competition result data
    GameResult game_result{};
    /// @brief Robot health data
    GameRobotHP game_robot_hp{};
    /// @brief Site event data
    EventData event_data{};
    /// @brief Action identifier data of the Official Projectile Supplier
    ProjectileSupplierStatus projectile_supplier_status{};
    /// @brief Referee warning data
    RefereeWarning referee_warning{};
    /// @brief Dart launching data
    DartStatus dart_status{};
    /// @brief Robot performance system data
    RobotPerformance robot_performance{};
    /// @brief Real-time chassis power and barrel heat data
    RobotPowerHeat robot_power_heat{};
    /// @brief Robot position data
    RobotPosition robot_position{};
    /// @brief Robot buff data
    RobotBuff robot_buff{};
    /// @brief Air support time data
    AirSupportStatus air_support_status{};
    /// @brief Damage status data
    DamageStatus damage_status{};
    /// @brief Real-time launching data
    LaunchingStatus launching_status{};
    /// @brief Projectile allowance data
    ProjectileAllowance projectile_allowance{};
    /// @brief RFID status data
    RFIDStatus rfid_status{};
    /// @brief Dart command data
    DartCommand dart_command{};
    /// @brief Ground robot positions data
    GroundRobotPositions ground_robot_positions{};
    /// @brief Radar progress data
    RadarProgress radar_progress{};
    /// @brief Sentry decision data
    SentryDecision sentry_decision{};
    /// @brief Radar decision data
    RadarDecision radar_decision{};
    /// @brief Robot interaction data
    RobotInteraction robot_interaction{};
    /// @brief Data about the interaction between the Custom Controller and robots
    CustomControllerRobot custom_controller_robot{};
    /// @brief Player client's small map interaction data
    SmallMapCommand small_map_command{};
    /// @brief Keyboard, mouse, and remote control data
    KBMInteraction kbm_interaction{};
    /// @brief Radar data received by player clients' Small Maps
    SmallMapRadarPosition small_map_radar_position{};
    /// @brief Data about the interaction between the Custom Controller and player clients
    CustomControllerClient custom_controller_client{};
    /// @brief Sentry data received by player clients' Small Maps
    SmallMapSentryCommand small_map_sentry_command{};
    /// @brief Robot data received by player clients' Small Map
    SmallMapRobotData small_map_robot_data{};
};

/// @brief Game Staus packet offset for comms
constexpr uint32_t REF_COMMS_GAME_STATUS_OFFSET = 0;
/// @brief Game Result packet offset for comms
constexpr uint32_t REF_COMMS_GAME_RESULT_OFFSET = 11;
/// @brief Game Robot HP packet offset for comms
constexpr uint32_t REF_COMMS_GAME_ROBOT_HP_OFFSET = 12;
/// @brief Event Data packet offset for comms
constexpr uint32_t REF_COMMS_EVENT_DATE_OFFSET = 44;
/// @brief Projectile Supplier Status packet offset for comms
constexpr uint32_t REF_COMMS_PROJECTILE_SUPPLIER_STATUS_OFFSET = 48;
/// @brief Referee Warning packet offset for comms
constexpr uint32_t REF_COMMS_REFEREE_WARNING_OFFSET = 52;
/// @brief Dart Status packet offset for comms
constexpr uint32_t REF_COMMS_ROBOT_PERFORMANCE_OFFSET = 55;
/// @brief Robot Power Heat packet offset for comms
constexpr uint32_t REF_COMMS_ROBOT_POWER_HEAT_OFFSET = 68;
/// @brief Robot Position packet offset for comms
constexpr uint32_t REF_COMMS_ROBOT_POSITION_OFFSET = 84;
/// @brief Robot Buff packet offset for comms
constexpr uint32_t REF_COMMS_ROBOT_BUFF_OFFSET = 100;
/// @brief Damage Status packet offset for comms
constexpr uint32_t REF_COMMS_DAMAGE_STATUS_OFFSET = 106;
/// @brief Launching Status packet offset for comms
constexpr uint32_t REF_COMMS_LAUNCHING_STATUS_OFFSET = 107;
/// @brief Projectile Allowance packet offset for comms
constexpr uint32_t REF_COMMS_PROJECTILE_ALLOWANCE_OFFSET = 114;
/// @brief RFID Status packet offset for comms
constexpr uint32_t REF_COMMS_RFID_STATUS_OFFSET = 120;
/// @brief KBM Interaction packet offset for comms
constexpr uint32_t REF_COMMS_KBM_INTERACTION_OFFSET = 124;
/// @brief End of the Ref Data packet for comms
constexpr uint32_t REF_COMMS_END_OFFSET = 136;

#endif // REF_SYSTEM_PACKET_DEFINITIONS_HPP