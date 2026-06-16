#pragma once

#include "comms/data/comms_ref_data.hpp"
#include <Arduino.h>

/// @brief Maximum supported Ref System frame data length.
/// @note Command ID 0x0301 is the largest packet that we use (118 bytes).
constexpr uint16_t REF_MAX_DATA_SIZE = 118;

/// @brief Maximum size of a Ref System packet in bytes \n
/// @brief 5 byte header + 2 byte command ID + max supported data segment + 2 byte CRC
constexpr uint16_t REF_MAX_PACKET_SIZE = 5 + 2 + REF_MAX_DATA_SIZE + 2;

/// @brief Maximum valid command ID for Ref System packets
constexpr uint16_t REF_MAX_COMMAND_ID = 0x0311;

/// @brief Size of a VT03/VT13 remote-control data frame in bytes
constexpr uint8_t VTM_REMOTE_CONTROL_PACKET_SIZE = 21;
/// @brief First fixed byte of a VT03/VT13 remote-control data frame
constexpr uint8_t VTM_REMOTE_CONTROL_HEADER_1 = 0xA9;
/// @brief Second fixed byte of a VT03/VT13 remote-control data frame
constexpr uint8_t VTM_REMOTE_CONTROL_HEADER_2 = 0x53;
/// @brief Maximum age before VTM control input is treated as stale
constexpr uint32_t VTM_REMOTE_CONTROL_TIMEOUT_MS = 250;

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
    /// @brief Radar data received by player clients' Small Maps
    SMALL_MAP_RADAR_POSITION = 0x0305,
    /// @brief Data about the interaction between the Custom Controller and player clients
    CUSTOM_CONTROLLER_CLIENT = 0x0306,
    /// @brief Sentry data received by player clients' Small Maps
    SMALL_MAP_SENTRY_COMMAND = 0x0307,
    /// @brief Robot data received by player clients' Small Map
    SMALL_MAP_ROBOT_DATA = 0x0308,
    /// @brief Unsupported custom data sent from a robot to the Custom Controller.
    ROBOT_CUSTOM_CONTROLLER_DATA = 0x0309,
    /// @brief Unsupported custom data sent from a robot to the Custom Client.
    ROBOT_CUSTOM_CLIENT_DATA = 0x0310,
    /// @brief Unsupported custom command sent from the Custom Client to robots.
    CUSTOM_CLIENT_ROBOT_COMMAND = 0x0311,
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
    void print() const {
        Serial.printf("\tSOF: %x\n", SOF);
        Serial.printf("\tLength: %u\n", data_length);
        Serial.printf("\tSequence: %u\n", sequence);
        Serial.printf("\tCRC: %x\n", CRC);
    }
};

/// @brief Struct for the Frame data portion
struct FrameData {
    /// @brief Data array to hold the Frame data portion
    uint8_t data[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Helpful index operator. Allows array-like indexing from the object itself
    /// @param index index
    /// @return uint8_t data at index
    uint8_t operator[](int index) { return data[index]; }
    /// @brief Helpful index operator. Allows const array-like indexing from the object itself
    /// @param index index
    /// @return uint8_t data at index
    uint8_t operator[](int index) const { return data[index]; }
};

/// @brief Struct for the entire Frame. Consists of a FrameHeader, Command ID, FrameData, and CRC
struct Frame {
    /// @brief Header portion of a Frame
    FrameHeader header{};
    /// @brief Command ID potion of a Frame
    uint16_t commandID = 0;
    /// @brief Data portion of a Frame
    FrameData data{};
    /// @brief 16-bit CRC for the entire Frame
    uint16_t CRC = 0;

    /// @brief Prints the Frame
    void print() const {
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
    /// @brief 1: RMUC. 2: Reserved. 3: Reserved. 4: RMUL 3v3. 5: RUML 1v1.
    uint8_t competition_type : 4 = 0;
    /// @brief Current stage of the competition \n
    /// @brief 0: pre-competition. 1: preparation. 2: 15s Ref System initialization. 3: 5s countdown. 4: In competition. 5: Result calculation.
    uint8_t current_stage : 4 = 0;
    /// @brief Remaining time of the current round in seconds
    uint16_t round_time_remaining = 0;
    /// @brief UNIX time, effective after the robot is correctly connected to the Referee System's NTP server
    uint64_t unix_time = 0;

    /// @brief Prints the GameStatus packet
    void print() const {
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
        unix_time = 0;
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

    /// @brief Converts this GameStatus struct to a GameStatusData struct for comms transmission
    /// @return GameStatusData struct with the same data as this GameStatus struct
    GameStatusData to_comms_data() {
        GameStatusData data;
        data.competition_type = competition_type;
        data.current_stage = current_stage;
        data.round_time_remaining = round_time_remaining;
        data.unix_time = unix_time;
        return data;
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
    void print() const {
        Serial.println("GameResult:");
        Serial.printf("\tWinner: %u\n", winner);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        winner = data[0];
    }

    /// @brief Converts this GameResult struct to a GameResultData struct for comms transmission
    /// @return GameResultData struct with the same data as this GameResult struct
    GameResultData to_comms_data() {
        GameResultData data;
        data.winner = winner;
        return data;
    }
};

/// @brief Robot health data
/// @note transmitted at a fixed frequency of 3 Hz to all robots
/// @note ID: 0x0003
struct GameRobotHP {
    /// @brief Size of the GameRobotHP packet in bytes
    static const uint8_t packet_size = 16;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Health of own side's hero (Robot No. 1)
    uint16_t hero_health = 0;
    /// @brief Health of own side's engineer (Robot No. 2)
    uint16_t engineer_health = 0;
    /// @brief Health of own side's standard infantry (Robot No. 3)
    uint16_t standard_3_health = 0;
    /// @brief Health of own side's standard infantry (Robot No. 4)
    uint16_t standard_4_health = 0;
    /// @brief Reserved field
    uint16_t reserved = 0;
    /// @brief Health of own side's sentry (Robot No. 7)
    uint16_t sentry_health = 0;
    /// @brief Health of own side's outpost (Robot No. 8)
    uint16_t outpost_health = 0;
    /// @brief Health of own side's base (Robot No. 9)
    uint16_t base_health = 0;

    /// @brief Prints the GameRobotHP packet
    void print() const {
        Serial.println("GameRobotHP:");
        Serial.printf("\tHero: %u\n", hero_health);
        Serial.printf("\tEngineer: %u\n", engineer_health);
        Serial.printf("\tStandard 3: %u\n", standard_3_health);
        Serial.printf("\tStandard 4: %u\n", standard_4_health);
        Serial.printf("\tSentry: %u\n", sentry_health);
        Serial.printf("\tReserved: %u\n", reserved);
        Serial.printf("\tOutpost: %u\n", outpost_health);
        Serial.printf("\tBase: %u\n", base_health);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        hero_health = (data[1] << 8) | data[0];
        engineer_health = (data[3] << 8) | data[2];
        standard_3_health = (data[5] << 8) | data[4];
        standard_4_health = (data[7] << 8) | data[6];
        sentry_health = (data[11] << 8) | data[10];
        reserved = (data[9] << 8) | data[8];
        outpost_health = (data[13] << 8) | data[12];
        base_health = (data[15] << 8) | data[14];
    }

    /// @brief Converts this GameRobotHP struct to a RobotHealthData struct for comms transmission
    /// @return RobotHealthData struct with the same data as this GameRobotHP struct
    RobotHealthData to_comms_data() {
        RobotHealthData data;
        data.hero_health = hero_health;
        data.engineer_health = engineer_health;
        data.standard_3_health = standard_3_health;
        data.standard_4_health = standard_4_health;
        data.sentry_health = sentry_health;
        return data;
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
    /// @brief Reload zone status; 0 if no robot is in the reload zone, 1 if at least one robot is in the reload zone
    uint16_t reload_zone_status = 0;
    /// @brief Capture point status; 0 if uncaptured, 1 if captured by own team, 2 if captured by enemy team, 3 if captured by both teams
    uint16_t capture_point_status = 0;

    /// @brief Prints the EventData packet
    void print() const {
        Serial.println("EventData:");
        Serial.printf("\tReload Zone Status: %u\n", reload_zone_status);
        Serial.printf("\tCapture Point Status: %u\n", capture_point_status);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        reload_zone_status = (data[0] >> 2) & 0x01;                               // bit 2
        capture_point_status = ((data[2] >> 7) & 0x01) | ((data[3] & 0x01) << 1); // bits 23 and 24
    }

    /// @brief Converts this EventData struct to a GameEventData struct for comms transmission
    /// @return GameEventData struct with the same data as this EventData struct
    GameEventData to_comms_data() {
        GameEventData data;
        data.reload_zone_status = reload_zone_status;
        data.capture_point_status = capture_point_status;
        return data;
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
    void print() const {
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
    /// @brief 0: Default. 1: Outpost. 2: Fixed target in Base. 3: Random fixed target in Base. 4: Random moving target in Base. 5: Terminal moving target in Base.
    uint16_t target_last_hit : 3;
    /// @brief Total number of recent hits to a target in the opponent team
    /// @brief 0: Default. 4: Max.
    uint16_t num_recent_hits : 3;
    /// @brief Target currently selected to be hit by the dart.
    /// @brief 0: No target or Outpost. 1: Fixed target in Base. 2: Random fixed target in Base. 3: Random moving target in Base. 4: Terminal moving target in Base.
    uint16_t current_target : 3;
    /// @brief Reserved.
    uint16_t reserved : 7;

    /// @brief Prints the DartStatus packet
    void print() const {
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
        uint16_t dart_info = (data[2] << 8) | data[1];
        target_last_hit = dart_info & 0x07;
        num_recent_hits = (dart_info >> 3) & 0x07;
        current_target = (dart_info >> 6) & 0x07;
        reserved = (dart_info >> 9) & 0x7F;
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
    uint8_t gimbal_power_active : 1;
    /// @brief Whether chassis line is powered
    uint8_t chassis_power_active : 1;
    /// @brief Whether shooter line is powered
    uint8_t shooter_power_active : 1;

    /// @brief Prints the RobotPerformance packet
    void print() const {
        Serial.println("RobotPerformance:");
        Serial.printf("\tRobot ID: %u\n", robot_ID);
        Serial.printf("\tRobot Level: %u\n", robot_level);
        Serial.printf("\tCurrent HP: %u\n", current_HP);
        Serial.printf("\tMax HP: %u\n", max_HP);
        Serial.printf("\tBarrel Cooling Rate: %u\n", barrel_cooling_rate);
        Serial.printf("\tBarrel Heat Limit: %u\n", barrel_heat_limit);
        Serial.printf("\tChassis Power Limit: %u\n", chassis_power_limit);
        Serial.printf("\tGimbal Power Active: %u\n", gimbal_power_active);
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
        gimbal_power_active = data[12] & 0x01;
        chassis_power_active = (data[12] >> 1) & 0x01;
        shooter_power_active = (data[12] >> 2) & 0x01;
    }

    /// @brief Converts this RobotPerformance struct to a RobotPerformanceData struct for comms transmission
    /// @return RobotPerformanceData struct with the same data as this RobotPerformance struct
    RobotPerformanceData to_comms_data() {
        RobotPerformanceData data;
        data.robot_id = robot_ID;
        data.robot_level = robot_level;
        data.current_health = current_HP;
        data.max_health = max_HP;
        data.barrel_cooling_rate = barrel_cooling_rate;
        data.barrel_heat_limit = barrel_heat_limit;
        data.chassis_power_limit = chassis_power_limit;
        data.gimbal_power_active = gimbal_power_active;
        data.chassis_power_active = chassis_power_active;
        data.shooter_power_active = shooter_power_active;
        return data;
    }
};

/// @brief Real-time chassis power and barrel heat data
/// @note transmitted at a fixed frequency of 10 Hz to a specific robot
/// @note ID: 0x0202
struct RobotPowerHeat {
    /// @brief Size of the RobotPower packet in bytes
    static const uint8_t packet_size = 14;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Reserved
    uint16_t reserved_1 = 0;
    /// @brief Reserved
    uint16_t reserved_2 = 0;
    /// @brief Reserved
    float reserved_3 = 0.f;
    /// @brief Buffer energy; unit: J
    uint16_t buffer_energy = 0;
    /// @brief Barrel heat of the 1st 17mm Launching Mechanism
    uint16_t barrel_heat_17mm = 0;
    /// @brief Barrel heat of the 42mm Launching Mechanism
    uint16_t barrel_heat_42mm = 0;

    /// @brief Prints the RobotPowerHeat packet
    void print() const {
        Serial.println("RobotPowerHeat:");
        Serial.printf("\tReserved 1: %u\n", reserved_1);
        Serial.printf("\tReserved 2: %u\n", reserved_2);
        Serial.printf("\tReserved 3: %f\n", reserved_3);
        Serial.printf("\tBuffer Energy: %u\n", buffer_energy);
        Serial.printf("\tBarrel Heat 1 17mm: %u\n", barrel_heat_17mm);
        Serial.printf("\tBarrel Heat 42mm: %u\n", barrel_heat_42mm);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        reserved_1 = (data[1] << 8) | data[0];
        reserved_2 = (data[3] << 8) | data[2];

        uint32_t reserved_3_raw = (static_cast<uint32_t>(data[7]) << 24) | (static_cast<uint32_t>(data[6]) << 16) | (static_cast<uint32_t>(data[5]) << 8) | static_cast<uint32_t>(data[4]);
        memcpy(&reserved_3, &reserved_3_raw, sizeof(reserved_3));

        buffer_energy = (data[9] << 8) | data[8];
        barrel_heat_17mm = (data[11] << 8) | data[10];
        barrel_heat_42mm = (data[13] << 8) | data[12];
    }

    /// @brief Converts this RobotPowerHeat struct to a RobotPowerHeatData struct for comms transmission
    /// @return RobotPowerHeatData struct with the same data as this RobotPowerHeat struct
    RobotPowerHeatData to_comms_data() {
        RobotPowerHeatData data;
        data.buffer_energy = buffer_energy;
        data.barrel_heat_17mm = barrel_heat_17mm;
        data.barrel_heat_42mm = barrel_heat_42mm;
        return data;
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
    void print() const {
        Serial.println("RobotPosition:");
        Serial.printf("\tX: %f\n", x);
        Serial.printf("\tY: %f\n", y);
        Serial.printf("\tAngle: %f\n", angle);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);

        uint32_t x_raw = (static_cast<uint32_t>(data[3]) << 24) | (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[1]) << 8) | static_cast<uint32_t>(data[0]);
        memcpy(&x, &x_raw, sizeof(x));
        uint32_t y_raw = (static_cast<uint32_t>(data[7]) << 24) | (static_cast<uint32_t>(data[6]) << 16) | (static_cast<uint32_t>(data[5]) << 8) | static_cast<uint32_t>(data[4]);
        memcpy(&y, &y_raw, sizeof(y));
        uint32_t angle_raw = (static_cast<uint32_t>(data[11]) << 24) | (static_cast<uint32_t>(data[10]) << 16) | (static_cast<uint32_t>(data[9]) << 8) | static_cast<uint32_t>(data[8]);
        memcpy(&angle, &angle_raw, sizeof(angle));
    }
};

/// @brief Robot buff data
/// @note transmitted at a fixed frequency of 3 Hz to a specific robot
/// @note ID: 0x0204
struct RobotBuff {
    /// @brief Size of the RobotBuff packet in bytes
    static const uint8_t packet_size = 8;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Robot's HP recovery buff (in percentage; a value of 10 indicates that HP recovery per second is 10% of the maximum HP.)
    uint8_t hp_recovery = 0;
    /// @brief Robot's barrel cooling rate (in absolute value; a value of 5 indicates a cooling rate of 5 times.)
    uint16_t heat_cooling = 0;
    /// @brief Robot's defense buff (in percentage; a value of 50 indicates a defense buff of 50%.)
    uint8_t defence = 0;
    /// @brief Robot's negative defense buff (in percentage; a value of 30 indicates a defense buff of -30%.)
    uint8_t negative_defence = 0;
    /// @brief Robot's attack buff (in percentage; a value of 50 indicates an attack buff of 50%.)
    uint16_t attack = 0;
    /// @brief Remaining energy value feedback.
    uint8_t remaining_energy = 0;

    /// @brief Prints the RobotBuff packet
    void print() const {
        Serial.println("RobotBuff:");
        Serial.printf("\tHP Recovery: %u\n", hp_recovery);
        Serial.printf("\tHeat Cooling: %u\n", heat_cooling);
        Serial.printf("\tDefence: %u\n", defence);
        Serial.printf("\tNegative Defence: %u\n", negative_defence);
        Serial.printf("\tAttack: %u\n", attack);
        Serial.printf("\tRemaining Energy: %u\n", remaining_energy);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        hp_recovery = data[0];
        heat_cooling = (data[2] << 8) | data[1];
        defence = data[3];
        negative_defence = data[4];
        attack = (data[6] << 8) | data[5];
        remaining_energy = data[7];
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
    void print() const {
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
    /// @brief Converts this DamageStatus struct to a DamageStatusData struct for comms transmission
    /// @return DamageStatusData struct with the same data as this DamageStatus struct
    DamageStatusData to_comms_data() {
        DamageStatusData data;
        data.armor_plate_ID = armor_plate_ID;
        data.damage_type = damage_type;
        return data;
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
    void print() const {
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
        uint32_t initial_speed_raw = (static_cast<uint32_t>(data[6]) << 24) | (static_cast<uint32_t>(data[5]) << 16) | (static_cast<uint32_t>(data[4]) << 8) | static_cast<uint32_t>(data[3]);
        memcpy(&initial_speed, &initial_speed_raw, sizeof(initial_speed));
    }

    /// @brief Converts this LaunchingStatus struct to a LaunchingStatusData struct for comms transmission
    /// @return LaunchingStatusData struct with the same data as this LaunchingStatus struct
    LaunchingStatusData to_comms_data() {
        LaunchingStatusData data;
        data.projectile_type = projectile_type;
        data.launching_mechanism = launching_mechanism;
        data.launching_frequency = launching_frequency;
        data.initial_speed = initial_speed;
        return data;
    }
};

/// @brief Projectile allowance data
/// @note transmitted at a fixed frequency of 10 Hz to a specific robot
/// @note ID: 0x0208
struct ProjectileAllowance {
    /// @brief Size of the ProjectileAllowance packet in bytes
    static const uint8_t packet_size = 8;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Number of 17mm projectiles remaining
    uint16_t num_17mm = 0;
    /// @brief Number of 42mm projectiles remaining
    uint16_t num_42mm = 0;
    /// @brief Num Gold Coins remaining
    uint16_t num_gold = 0;
    /// @brief Reserved 17mm projectile allowance provided by Fortress Buff Point
    uint16_t num_17mm_fortress = 0;

    /// @brief Prints the ProjectileAllowance packet
    void print() const {
        Serial.println("ProjectileAllowance:");
        Serial.printf("\tNum 17mm: %u\n", num_17mm);
        Serial.printf("\tNum 42mm: %u\n", num_42mm);
        Serial.printf("\tNum Gold: %u\n", num_gold);
        Serial.printf("\tNum 17mm Fortress: %u\n", num_17mm_fortress);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        num_17mm = (data[1] << 8) | data[0];
        num_42mm = (data[3] << 8) | data[2];
        num_gold = (data[5] << 8) | data[4];
        num_17mm_fortress = (data[7] << 8) | data[6];
    }

    /// @brief Converts this ProjectileAllowance struct to a ProjectileAllowanceData struct for comms transmission
    /// @return ProjectileAllowanceData struct with the same data as this ProjectileAllowance struct
    ProjectileAllowanceData to_comms_data() {
        ProjectileAllowanceData data;
        data.num_17mm = num_17mm;
        data.num_42mm = num_42mm;
        data.num_gold = num_gold;
        return data;
    }
};

/// @brief Robot RFID module status
/// @note transmitted at a fixed frequency of 3 Hz to a our robots
/// @note ID: 0x0209
struct RFIDStatus {
    /// @brief Size of the RFIDStatus packet in bytes
    static const uint8_t packet_size = 5;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    // Meaning of bit value 0 or 1: whether the Buff Point's RFID card is detected.

    /// @brief own side's Base Buff Point
    uint32_t our_base_buff_point : 1;
    /// @brief Own Side Central Elevated Ground Buff Point
    uint32_t our_central_elevated_ground : 1;
    /// @brief Opponent Central Elevated Ground Buff Point
    uint32_t their_central_elevated_ground : 1;
    /// @brief Own Side Trapezoid-Shaped Elevated Ground Buff Point
    uint32_t our_trapezoid_buff_point : 1;
    /// @brief Opponent Trapezoid-Shaped Elevated Ground Buff Point
    uint32_t their_trapezoid_buff_point : 1;
    /// @brief Own Side Terrain Crossing Buff Point near own launch ramp, before the ramp
    uint32_t our_launch_ramp_buff_point_front : 1;
    /// @brief Own Side Terrain Crossing Buff Point after own launch ramp
    uint32_t our_launch_ramp_buff_point_back : 1;
    /// @brief Opponent Terrain Crossing Buff Point before opponent launch ramp
    uint32_t their_launch_ramp_buff_point_front : 1;
    /// @brief Opponent Terrain Crossing Buff Point after opponent launch ramp
    uint32_t their_launch_ramp_buff_point_back : 1;
    /// @brief Own Side Terrain Crossing Buff Point below trapezoid center
    uint32_t our_trapezoid_center_crossing : 1;
    /// @brief Own Side Terrain Crossing Buff Point above central elevated ground
    uint32_t our_central_elevated_crossing : 1;
    /// @brief Opponent Terrain Crossing Buff Point below central elevated ground
    uint32_t their_central_elevated_lower_crossing : 1;
    /// @brief Opponent Terrain Crossing Buff Point above central elevated ground
    uint32_t their_central_elevated_upper_crossing : 1;
    /// @brief Own Side Terrain Crossing Buff Point below road
    uint32_t our_road_lower_crossing : 1;
    /// @brief Own Side Terrain Crossing Buff Point above road
    uint32_t our_road_upper_crossing : 1;
    /// @brief Opponent Terrain Crossing Buff Point below road
    uint32_t their_road_lower_crossing : 1;
    /// @brief Opponent Terrain Crossing Buff Point above road
    uint32_t their_road_upper_crossing : 1;
    /// @brief Own Side Fortress Buff Point
    uint32_t our_fortress_buff_point : 1;
    /// @brief Own Side Outpost Buff Point
    uint32_t our_outpost_buff_point : 1;
    /// @brief Own Side Resupply Zone not overlapping with Resource Zone/RMUL Resupply Zone
    uint32_t our_resupply_zone : 1;
    /// @brief Own Side Resupply Zone overlapping with Resource Zone
    uint32_t our_overlapping_resupply_zone : 1;
    /// @brief Own Side Assembly Buff Point
    uint32_t our_assembly_buff_point : 1;
    /// @brief Opponent Assembly Buff Point
    uint32_t their_assembly_buff_point : 1;
    /// @brief Central Buff Point (for RMUL only)
    uint32_t central_buff_point : 1;
    /// @brief Opponent Fortress Buff Point
    uint32_t their_fortress_buff_point : 1;
    /// @brief Opponent Outpost Buff Point
    uint32_t their_outpost_buff_point : 1;
    /// @brief Own Side tunnel lower road crossing
    uint32_t our_tunnel_road_lower : 1;
    /// @brief Own Side tunnel middle road crossing
    uint32_t our_tunnel_road_middle : 1;
    /// @brief Own Side tunnel upper road crossing
    uint32_t our_tunnel_road_upper : 1;
    /// @brief Own Side tunnel lower trapezoid crossing
    uint32_t our_tunnel_trapezoid_lower : 1;
    /// @brief Own Side tunnel middle trapezoid crossing
    uint32_t our_tunnel_trapezoid_middle : 1;
    /// @brief Own Side tunnel upper trapezoid crossing
    uint32_t our_tunnel_trapezoid_upper : 1;
    /// @brief Opponent tunnel lower road crossing
    uint8_t their_tunnel_road_lower : 1;
    /// @brief Opponent tunnel middle road crossing
    uint8_t their_tunnel_road_middle : 1;
    /// @brief Opponent tunnel upper road crossing
    uint8_t their_tunnel_road_upper : 1;
    /// @brief Opponent tunnel lower trapezoid crossing
    uint8_t their_tunnel_trapezoid_lower : 1;
    /// @brief Opponent tunnel middle trapezoid crossing
    uint8_t their_tunnel_trapezoid_middle : 1;
    /// @brief Opponent tunnel upper trapezoid crossing
    uint8_t their_tunnel_trapezoid_upper : 1;
    /// @brief Unused bits in the RFIDStatus packet
    uint8_t reserved : 2;

    /// @brief Prints the RFIDStatus packet
    /// @todo Implement this before china
    void print() const {}

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        uint32_t rfid_status = (static_cast<uint32_t>(data[3]) << 24) | (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[1]) << 8) | static_cast<uint32_t>(data[0]);
        our_base_buff_point = rfid_status & 0x01;
        our_central_elevated_ground = (rfid_status >> 1) & 0x01;
        their_central_elevated_ground = (rfid_status >> 2) & 0x01;
        our_trapezoid_buff_point = (rfid_status >> 3) & 0x01;
        their_trapezoid_buff_point = (rfid_status >> 4) & 0x01;
        our_launch_ramp_buff_point_front = (rfid_status >> 5) & 0x01;
        our_launch_ramp_buff_point_back = (rfid_status >> 6) & 0x01;
        their_launch_ramp_buff_point_front = (rfid_status >> 7) & 0x01;
        their_launch_ramp_buff_point_back = (rfid_status >> 8) & 0x01;
        our_trapezoid_center_crossing = (rfid_status >> 9) & 0x01;
        our_central_elevated_crossing = (rfid_status >> 10) & 0x01;
        their_central_elevated_lower_crossing = (rfid_status >> 11) & 0x01;
        their_central_elevated_upper_crossing = (rfid_status >> 12) & 0x01;
        our_road_lower_crossing = (rfid_status >> 13) & 0x01;
        our_road_upper_crossing = (rfid_status >> 14) & 0x01;
        their_road_lower_crossing = (rfid_status >> 15) & 0x01;
        their_road_upper_crossing = (rfid_status >> 16) & 0x01;
        our_fortress_buff_point = (rfid_status >> 17) & 0x01;
        our_outpost_buff_point = (rfid_status >> 18) & 0x01;
        our_resupply_zone = (rfid_status >> 19) & 0x01;
        our_overlapping_resupply_zone = (rfid_status >> 20) & 0x01;
        our_assembly_buff_point = (rfid_status >> 21) & 0x01;
        their_assembly_buff_point = (rfid_status >> 22) & 0x01;
        central_buff_point = (rfid_status >> 23) & 0x01;
        their_fortress_buff_point = (rfid_status >> 24) & 0x01;
        their_outpost_buff_point = (rfid_status >> 25) & 0x01;
        our_tunnel_road_lower = (rfid_status >> 26) & 0x01;
        our_tunnel_road_middle = (rfid_status >> 27) & 0x01;
        our_tunnel_road_upper = (rfid_status >> 28) & 0x01;
        our_tunnel_trapezoid_lower = (rfid_status >> 29) & 0x01;
        our_tunnel_trapezoid_middle = (rfid_status >> 30) & 0x01;
        our_tunnel_trapezoid_upper = (rfid_status >> 31) & 0x01;
        their_tunnel_road_lower = data[4] & 0x01;
        their_tunnel_road_middle = (data[4] >> 1) & 0x01;
        their_tunnel_road_upper = (data[4] >> 2) & 0x01;
        their_tunnel_trapezoid_lower = (data[4] >> 3) & 0x01;
        their_tunnel_trapezoid_middle = (data[4] >> 4) & 0x01;
        their_tunnel_trapezoid_upper = (data[4] >> 5) & 0x01;
        reserved = (data[4] >> 6) & 0x03;
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
    void print() const {
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
    /// @brief Reserved
    float reserved_1 = 0.f;
    /// @brief Reserved
    float reserved_2 = 0.f;

    /// @brief Prints the RobotPosition packet
    void print() const {
        Serial.println("RobotPosition:");
        Serial.printf("\tHero X: %f\n", hero_x);
        Serial.printf("\tHero Y: %f\n", hero_y);
        Serial.printf("\tEngineer X: %f\n", engineer_x);
        Serial.printf("\tEngineer Y: %f\n", engineer_y);
        Serial.printf("\tStandard 3 X: %f\n", standard_3_x);
        Serial.printf("\tStandard 3 Y: %f\n", standard_3_y);
        Serial.printf("\tStandard 4 X: %f\n", standard_4_x);
        Serial.printf("\tStandard 4 Y: %f\n", standard_4_y);
        Serial.printf("\tReserved 1: %f\n", reserved_1);
        Serial.printf("\tReserved 2: %f\n", reserved_2);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);

        uint32_t hero_x_raw = (static_cast<uint32_t>(data[3]) << 24) | (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[1]) << 8) | static_cast<uint32_t>(data[0]);
        memcpy(&hero_x, &hero_x_raw, sizeof(hero_x));
        uint32_t hero_y_raw = (static_cast<uint32_t>(data[7]) << 24) | (static_cast<uint32_t>(data[6]) << 16) | (static_cast<uint32_t>(data[5]) << 8) | static_cast<uint32_t>(data[4]);
        memcpy(&hero_y, &hero_y_raw, sizeof(hero_y));
        uint32_t engineer_x_raw = (static_cast<uint32_t>(data[11]) << 24) | (static_cast<uint32_t>(data[10]) << 16) | (static_cast<uint32_t>(data[9]) << 8) | static_cast<uint32_t>(data[8]);
        memcpy(&engineer_x, &engineer_x_raw, sizeof(engineer_x));
        uint32_t engineer_y_raw = (static_cast<uint32_t>(data[15]) << 24) | (static_cast<uint32_t>(data[14]) << 16) | (static_cast<uint32_t>(data[13]) << 8) | static_cast<uint32_t>(data[12]);
        memcpy(&engineer_y, &engineer_y_raw, sizeof(engineer_y));
        uint32_t standard_3_x_raw = (static_cast<uint32_t>(data[19]) << 24) | (static_cast<uint32_t>(data[18]) << 16) | (static_cast<uint32_t>(data[17]) << 8) | static_cast<uint32_t>(data[16]);
        memcpy(&standard_3_x, &standard_3_x_raw, sizeof(standard_3_x));
        uint32_t standard_3_y_raw = (static_cast<uint32_t>(data[23]) << 24) | (static_cast<uint32_t>(data[22]) << 16) | (static_cast<uint32_t>(data[21]) << 8) | static_cast<uint32_t>(data[20]);
        memcpy(&standard_3_y, &standard_3_y_raw, sizeof(standard_3_y));
        uint32_t standard_4_x_raw = (static_cast<uint32_t>(data[27]) << 24) | (static_cast<uint32_t>(data[26]) << 16) | (static_cast<uint32_t>(data[25]) << 8) | static_cast<uint32_t>(data[24]);
        memcpy(&standard_4_x, &standard_4_x_raw, sizeof(standard_4_x));
        uint32_t standard_4_y_raw = (static_cast<uint32_t>(data[31]) << 24) | (static_cast<uint32_t>(data[30]) << 16) | (static_cast<uint32_t>(data[29]) << 8) | static_cast<uint32_t>(data[28]);
        memcpy(&standard_4_y, &standard_4_y_raw, sizeof(standard_4_y));
        uint32_t reserved_1_raw = (static_cast<uint32_t>(data[35]) << 24) | (static_cast<uint32_t>(data[34]) << 16) | (static_cast<uint32_t>(data[33]) << 8) | static_cast<uint32_t>(data[32]);
        memcpy(&reserved_1, &reserved_1_raw, sizeof(reserved_1));
        uint32_t reserved_2_raw = (static_cast<uint32_t>(data[39]) << 24) | (static_cast<uint32_t>(data[38]) << 16) | (static_cast<uint32_t>(data[37]) << 8) | static_cast<uint32_t>(data[36]);
        memcpy(&reserved_2, &reserved_2_raw, sizeof(reserved_2));
    }
};

/// @brief Radar-marked progress data
/// @note transmitted at a fixed frequency of 1 Hz to our Radar
/// @note ID: 0x020C
struct RadarProgress {
    /// @brief Size of the RadarProgress packet in bytes
    static const uint8_t packet_size = 2;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Vulnerability status of Opponent Hero No. 1
    uint16_t opponent_hero : 1;
    /// @brief Vulnerability status of Opponent Engineer No. 2
    uint16_t opponent_engineer : 1;
    /// @brief Vulnerability status of Opponent Infantry No. 3
    uint16_t opponent_standard_3 : 1;
    /// @brief Vulnerability status of Opponent Infantry No. 4
    uint16_t opponent_standard_4 : 1;
    /// @brief Special marking status of Opponent Drone
    uint16_t opponent_drone : 1;
    /// @brief Vulnerability status of Opponent Sentry
    uint16_t opponent_sentry : 1;
    /// @brief Tracking status for Own Side Hero No. 1
    uint16_t own_hero : 1;
    /// @brief Tracking status for Own Side Engineer No. 2
    uint16_t own_engineer : 1;
    /// @brief Tracking status for Own Side Infantry No. 3
    uint16_t own_standard_3 : 1;
    /// @brief Tracking status for Own Side Infantry No. 4
    uint16_t own_standard_4 : 1;
    /// @brief Own Side Drone special marking status
    uint16_t own_drone : 1;
    /// @brief Own Side Sentry special status
    uint16_t own_sentry : 1;
    /// @brief Reserved
    uint16_t reserved : 4;

    /// @brief Prints the RadarProgress packet
    void print() const {
        Serial.println("RadarProgress:");
        Serial.printf("\tOpponent Hero: %u\n", opponent_hero);
        Serial.printf("\tOpponent Engineer: %u\n", opponent_engineer);
        Serial.printf("\tOpponent Standard 3: %u\n", opponent_standard_3);
        Serial.printf("\tOpponent Standard 4: %u\n", opponent_standard_4);
        Serial.printf("\tOpponent Drone: %u\n", opponent_drone);
        Serial.printf("\tOpponent Sentry: %u\n", opponent_sentry);
        Serial.printf("\tOwn Hero: %u\n", own_hero);
        Serial.printf("\tOwn Engineer: %u\n", own_engineer);
        Serial.printf("\tOwn Standard 3: %u\n", own_standard_3);
        Serial.printf("\tOwn Standard 4: %u\n", own_standard_4);
        Serial.printf("\tOwn Drone: %u\n", own_drone);
        Serial.printf("\tOwn Sentry: %u\n", own_sentry);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        uint16_t tracking_progress = (data[1] << 8) | data[0];
        opponent_hero = tracking_progress & 0x01;
        opponent_engineer = (tracking_progress >> 1) & 0x01;
        opponent_standard_3 = (tracking_progress >> 2) & 0x01;
        opponent_standard_4 = (tracking_progress >> 3) & 0x01;
        opponent_drone = (tracking_progress >> 4) & 0x01;
        opponent_sentry = (tracking_progress >> 5) & 0x01;
        own_hero = (tracking_progress >> 6) & 0x01;
        own_engineer = (tracking_progress >> 7) & 0x01;
        own_standard_3 = (tracking_progress >> 8) & 0x01;
        own_standard_4 = (tracking_progress >> 9) & 0x01;
        own_drone = (tracking_progress >> 10) & 0x01;
        own_sentry = (tracking_progress >> 11) & 0x01;
        reserved = (tracking_progress >> 12) & 0x0F;
    }
};

/// @brief Decision-making data of Sentry Robot
/// @note transmitted at a fixed frequency of 1 Hz to our Sentry
/// @note ID: 0x020D
struct SentryDecision {
    /// @brief Size of the SentryDecision packet in bytes
    static const uint8_t packet_size = 6;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @todo implement before china
    uint32_t sentry_info = 0;
    /// @todo implement before china
    uint16_t sentry_info_2 = 0;

    /// @brief Prints the SentryDecision packet
    void print() const {
        Serial.println("SentryDecision:");
        Serial.printf("\tSentry Info: %u\n", sentry_info);
        Serial.printf("\tSentry Info 2: %u\n", sentry_info_2);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        sentry_info = (static_cast<uint32_t>(data[3]) << 24) | (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[1]) << 8) | static_cast<uint32_t>(data[0]);
        sentry_info_2 = (static_cast<uint32_t>(data[5]) << 8) | static_cast<uint32_t>(data[4]);
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
    void print() const {
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
/// @note transmitted at a maximum frequency of 30 Hz when triggered by the sender
/// @note ID: 0x0301
struct RobotInteraction {
    /// @brief Size of the RobotInteraction packet in bytes
    static const uint8_t packet_size = 118;

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
    uint8_t data[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Prints the RobotInteraction packet
    void print() const {
        for (int i = 0; i < size; i++) {
            Serial.printf("%x ", data[i]);
        }
        Serial.println();
    }

    /// @brief Fills in this struct with the data from a Frame object
    /// @param frame Frame object to extract data from
    void set_data(Frame &frame) {
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
    uint8_t data[30] = {0};

    /// @brief Prints the ControllerRobots packet
    void print() const {
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
    /// @note The 2026, version 1.3.1 spec says this packet is 15 bytes, but its members sum to 12 bytes. This is likely a typo, but keep an eye out for issues with this packet size.
    static const uint8_t packet_size = 12;

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
    uint16_t cmd_source = 0;

    /// @brief Prints the SmallMapCommand packet
    void print() const {
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
        uint32_t target_position_x_raw = (static_cast<uint32_t>(data[3]) << 24) | (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[1]) << 8) | static_cast<uint32_t>(data[0]);
        memcpy(&target_position_x, &target_position_x_raw, sizeof(target_position_x));
        uint32_t target_position_y_raw = (static_cast<uint32_t>(data[7]) << 24) | (static_cast<uint32_t>(data[6]) << 16) | (static_cast<uint32_t>(data[5]) << 8) | static_cast<uint32_t>(data[4]);
        memcpy(&target_position_y, &target_position_y_raw, sizeof(target_position_y));
        cmd_keyboard = data[8];
        target_robot_id = data[9];
        cmd_source = (data[11] << 8) | data[10];
    }
};

/// @brief Radar data received by player clients' Small Maps
/// @note transmitted at a maximum frequency of 5 Hz to all of our player clients
/// @note ID: 0x0305
struct SmallMapRadarPosition {
    /// @brief Size of the SmallMapRadarPosition packet in bytes
    static const uint8_t packet_size = 48;

    /// @brief The raw byte array of data received from ref
    /// @note this is only the FrameData data rather than the whole ref packet
    uint8_t raw[REF_MAX_PACKET_SIZE] = {0};

    /// @brief Opponent Hero x-coordinate; unit: cm.
    uint16_t opponent_hero_x = 0;
    /// @brief Opponent Hero y-coordinate; unit: cm.
    uint16_t opponent_hero_y = 0;
    /// @brief Opponent Engineer x-coordinate; unit: cm.
    uint16_t opponent_engineer_x = 0;
    /// @brief Opponent Engineer y-coordinate; unit: cm.
    uint16_t opponent_engineer_y = 0;
    /// @brief Opponent Infantry No. 3 x-coordinate; unit: cm.
    uint16_t opponent_standard_3_x = 0;
    /// @brief Opponent Infantry No. 3 y-coordinate; unit: cm.
    uint16_t opponent_standard_3_y = 0;
    /// @brief Opponent Infantry No. 4 x-coordinate; unit: cm.
    uint16_t opponent_standard_4_x = 0;
    /// @brief Opponent Infantry No. 4 y-coordinate; unit: cm.
    uint16_t opponent_standard_4_y = 0;
    /// @brief Opponent Drone x-coordinate; unit: cm.
    uint16_t opponent_drone_x = 0;
    /// @brief Opponent Drone y-coordinate; unit: cm.
    uint16_t opponent_drone_y = 0;
    /// @brief Opponent Sentry x-coordinate; unit: cm.
    uint16_t opponent_sentry_x = 0;
    /// @brief Opponent Sentry y-coordinate; unit: cm.
    uint16_t opponent_sentry_y = 0;
    /// @brief Own side Hero x-coordinate; unit: cm.
    uint16_t own_hero_x = 0;
    /// @brief Own side Hero y-coordinate; unit: cm.
    uint16_t own_hero_y = 0;
    /// @brief Own side Engineer x-coordinate; unit: cm.
    uint16_t own_engineer_x = 0;
    /// @brief Own side Engineer y-coordinate; unit: cm.
    uint16_t own_engineer_y = 0;
    /// @brief Own side Infantry No. 3 x-coordinate; unit: cm.
    uint16_t own_standard_3_x = 0;
    /// @brief Own side Infantry No. 3 y-coordinate; unit: cm.
    uint16_t own_standard_3_y = 0;
    /// @brief Own side Infantry No. 4 x-coordinate; unit: cm.
    uint16_t own_standard_4_x = 0;
    /// @brief Own side Infantry No. 4 y-coordinate; unit: cm.
    uint16_t own_standard_4_y = 0;
    /// @brief Own side Drone x-coordinate; unit: cm.
    uint16_t own_drone_x = 0;
    /// @brief Own side Drone y-coordinate; unit: cm.
    uint16_t own_drone_y = 0;
    /// @brief Own side Sentry x-coordinate; unit: cm.
    uint16_t own_sentry_x = 0;
    /// @brief Own side Sentry y-coordinate; unit: cm.
    uint16_t own_sentry_y = 0;

    /// @brief Prints the SmallMapRadarPosition packet
    void print() const {
        Serial.println("SmallMapRadarPosition:");
        Serial.printf("\tOpponent Hero X: %u\n", opponent_hero_x);
        Serial.printf("\tOpponent Hero Y: %u\n", opponent_hero_y);
        Serial.printf("\tOpponent Engineer X: %u\n", opponent_engineer_x);
        Serial.printf("\tOpponent Engineer Y: %u\n", opponent_engineer_y);
        Serial.printf("\tOpponent Standard 3 X: %u\n", opponent_standard_3_x);
        Serial.printf("\tOpponent Standard 3 Y: %u\n", opponent_standard_3_y);
        Serial.printf("\tOpponent Standard 4 X: %u\n", opponent_standard_4_x);
        Serial.printf("\tOpponent Standard 4 Y: %u\n", opponent_standard_4_y);
        Serial.printf("\tOpponent Drone X: %u\n", opponent_drone_x);
        Serial.printf("\tOpponent Drone Y: %u\n", opponent_drone_y);
        Serial.printf("\tOpponent Sentry X: %u\n", opponent_sentry_x);
        Serial.printf("\tOpponent Sentry Y: %u\n", opponent_sentry_y);
        Serial.printf("\tOwn Hero X: %u\n", own_hero_x);
        Serial.printf("\tOwn Hero Y: %u\n", own_hero_y);
        Serial.printf("\tOwn Engineer X: %u\n", own_engineer_x);
        Serial.printf("\tOwn Engineer Y: %u\n", own_engineer_y);
        Serial.printf("\tOwn Standard 3 X: %u\n", own_standard_3_x);
        Serial.printf("\tOwn Standard 3 Y: %u\n", own_standard_3_y);
        Serial.printf("\tOwn Standard 4 X: %u\n", own_standard_4_x);
        Serial.printf("\tOwn Standard 4 Y: %u\n", own_standard_4_y);
        Serial.printf("\tOwn Drone X: %u\n", own_drone_x);
        Serial.printf("\tOwn Drone Y: %u\n", own_drone_y);
        Serial.printf("\tOwn Sentry X: %u\n", own_sentry_x);
        Serial.printf("\tOwn Sentry Y: %u\n", own_sentry_y);
    }

    /// @brief Fills in this struct with the data from a FrameData object
    /// @param data FrameData object to extract data from
    void set_data(FrameData data) {
        memcpy(raw, data.data, packet_size);
        opponent_hero_x = (data[1] << 8) | data[0];
        opponent_hero_y = (data[3] << 8) | data[2];
        opponent_engineer_x = (data[5] << 8) | data[4];
        opponent_engineer_y = (data[7] << 8) | data[6];
        opponent_standard_3_x = (data[9] << 8) | data[8];
        opponent_standard_3_y = (data[11] << 8) | data[10];
        opponent_standard_4_x = (data[13] << 8) | data[12];
        opponent_standard_4_y = (data[15] << 8) | data[14];
        opponent_drone_x = (data[17] << 8) | data[16];
        opponent_drone_y = (data[19] << 8) | data[18];
        opponent_sentry_x = (data[21] << 8) | data[20];
        opponent_sentry_y = (data[23] << 8) | data[22];
        own_hero_x = (data[25] << 8) | data[24];
        own_hero_y = (data[27] << 8) | data[26];
        own_engineer_x = (data[29] << 8) | data[28];
        own_engineer_y = (data[31] << 8) | data[30];
        own_standard_3_x = (data[33] << 8) | data[32];
        own_standard_3_y = (data[35] << 8) | data[34];
        own_standard_4_x = (data[37] << 8) | data[36];
        own_standard_4_y = (data[39] << 8) | data[38];
        own_drone_x = (data[41] << 8) | data[40];
        own_drone_y = (data[43] << 8) | data[42];
        own_sentry_x = (data[45] << 8) | data[44];
        own_sentry_y = (data[47] << 8) | data[46];
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
    void print() const {
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
        uint16_t mouse_x_data = (data[3] << 8) | data[2];
        uint16_t mouse_y_data = (data[5] << 8) | data[4];
        mouse_x = mouse_x_data & 0x0FFF;
        mouse_left = (mouse_x_data >> 12) & 0x0F;
        mouse_y = mouse_y_data & 0x0FFF;
        mouse_right = (mouse_y_data >> 12) & 0x0F;
        reserved = (data[7] << 8) | data[6];
    }
};

/// @brief Sentry data received by player clients' Small Maps
/// @note transmitted at a maximum frequency of 1 Hz to a specific player client
/// @note ID: 0x0307
struct SmallMapSentryCommand {
    /// @brief Size of the SmallMapSentryPosition packet in bytes
    static const uint8_t packet_size = 105;

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
    void print() const {
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
        sender_ID = (data[104] << 8) | data[103];
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
    void print() const {
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

/// @brief Remote-control data received from the VT03/VT13 receiver.
struct VTMRemoteControl {
    /// @brief The raw 21-byte VT03/VT13 remote-control frame
    uint8_t raw[VTM_REMOTE_CONTROL_PACKET_SIZE] = {0};

    /// @brief The horizontal position of the receiver's right control stick
    uint16_t channel_0 = 1024;
    /// @brief The vertical position of the receiver's right control stick
    uint16_t channel_1 = 1024;
    /// @brief The vertical position of the receiver's left control stick
    uint16_t channel_2 = 1024;
    /// @brief The horizontal position of the receiver's left control stick
    uint16_t channel_3 = 1024;
    /// @brief Receiver mode switch. C: 0, N: 1, S: 2.
    uint8_t mode_switch = 0;
    /// @brief Whether the receiver's pause button is pressed
    bool pause_button = false;
    /// @brief Whether the receiver's left customizable button is pressed
    bool custom_button_left = false;
    /// @brief Whether the receiver's right customizable button is pressed
    bool custom_button_right = false;
    /// @brief The position of the receiver's dial
    uint16_t dial = 1024;
    /// @brief Whether the receiver's trigger is pressed
    bool trigger = false;
    /// @brief x-axis moving speed of the mouse. A negative value indicates a left movement.
    int16_t mouse_speed_x = 0;
    /// @brief y-axis moving speed of the mouse. A negative value indicates a backward movement.
    int16_t mouse_speed_y = 0;
    /// @brief Scroll wheel's moving speed of the mouse. A negative value indicates a backward movement.
    int16_t scroll_speed = 0;
    /// @brief Whether the mouse's left button is pressed.
    bool button_left = false;
    /// @brief Whether the mouse's right button is pressed.
    bool button_right = false;
    /// @brief Whether the mouse's middle button is pressed.
    bool button_middle = false;
    /// @brief Keyboard key bitmask.
    uint16_t keyboard_value = 0;
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
    /// @brief Time of the last valid packet update.
    uint32_t last_update_ms = 0;

    /// @brief Clears control inputs while preserving packet timing metadata.
    void clear() {
        channel_0 = 1024; // 1024 is center
        channel_1 = 1024;
        channel_2 = 1024;
        channel_3 = 1024;
        mode_switch = 0;
        pause_button = false;
        custom_button_left = false;
        custom_button_right = false;
        dial = 1024;
        trigger = false;
        mouse_speed_x = 0;
        mouse_speed_y = 0;
        scroll_speed = 0;
        button_left = false;
        button_right = false;
        button_middle = false;
        set_keyboard_value(0);
    }

    /// @brief Whether this input was updated recently enough to consume.
    /// @return True if this input has a nonzero update timestamp within the timeout.
    bool is_fresh() const { return last_update_ms != 0 && millis() - last_update_ms <= VTM_REMOTE_CONTROL_TIMEOUT_MS; }

    /// @brief Fills in this struct with data from a VT03/VT13 remote-control frame.
    /// @param packet Raw 21-byte frame
    void set_data(const uint8_t packet[VTM_REMOTE_CONTROL_PACKET_SIZE]) {
        memcpy(raw, packet, VTM_REMOTE_CONTROL_PACKET_SIZE);

        channel_0 = read_bits(packet, 16, 11);
        channel_1 = read_bits(packet, 27, 11);
        channel_2 = read_bits(packet, 38, 11);
        channel_3 = read_bits(packet, 49, 11);
        mode_switch = read_bits(packet, 60, 2);
        pause_button = read_bits(packet, 62, 1);
        custom_button_left = read_bits(packet, 63, 1);
        custom_button_right = read_bits(packet, 64, 1);
        dial = read_bits(packet, 65, 11);
        trigger = read_bits(packet, 76, 1);
        mouse_speed_x = static_cast<int16_t>(read_bits(packet, 80, 16));
        mouse_speed_y = static_cast<int16_t>(read_bits(packet, 96, 16));
        scroll_speed = static_cast<int16_t>(read_bits(packet, 112, 16));
        button_left = read_bits(packet, 128, 2);
        button_right = read_bits(packet, 130, 2);
        button_middle = read_bits(packet, 132, 2);
        set_keyboard_value(read_bits(packet, 136, 16));
        last_update_ms = millis();
    }

  private:
    /// @brief Reads a little-endian bit field from a VT03/VT13 remote-control frame.
    /// @param packet Raw 21-byte frame
    /// @param bit_offset Offset of the first bit to read
    /// @param bit_length Number of bits to read
    /// @return Extracted bit-field value
    static uint16_t read_bits(const uint8_t packet[VTM_REMOTE_CONTROL_PACKET_SIZE], uint16_t bit_offset, uint8_t bit_length) {
        uint16_t value = 0;
        for (uint8_t i = 0; i < bit_length; i++) {
            uint16_t source_bit = bit_offset + i;
            value |= static_cast<uint16_t>((packet[source_bit / 8] >> (source_bit % 8)) & 0x01) << i;
        }
        return value;
    }

    /// @brief Stores the keyboard bitmask and expands it into individual key fields.
    /// @param value Keyboard key bitmask from the VT03/VT13 remote-control frame
    void set_keyboard_value(uint16_t value) {
        keyboard_value = value;
        key_w = value & 0x01;
        key_s = (value >> 1) & 0x01;
        key_a = (value >> 2) & 0x01;
        key_d = (value >> 3) & 0x01;
        key_shift = (value >> 4) & 0x01;
        key_ctrl = (value >> 5) & 0x01;
        key_q = (value >> 6) & 0x01;
        key_e = (value >> 7) & 0x01;
        key_r = (value >> 8) & 0x01;
        key_f = (value >> 9) & 0x01;
        key_g = (value >> 10) & 0x01;
        key_z = (value >> 11) & 0x01;
        key_x = (value >> 12) & 0x01;
        key_c = (value >> 13) & 0x01;
        key_v = (value >> 14) & 0x01;
        key_b = (value >> 15) & 0x01;
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
    /// @brief Radar data received by player clients' Small Maps
    SmallMapRadarPosition small_map_radar_position{};
    /// @brief Data about the interaction between the Custom Controller and player clients
    CustomControllerClient custom_controller_client{};
    /// @brief Sentry data received by player clients' Small Maps
    SmallMapSentryCommand small_map_sentry_command{};
    /// @brief Robot data received by player clients' Small Map
    SmallMapRobotData small_map_robot_data{};
    /// @brief Keyboard and mouse control data received from the official Player's Client through VTM.
    VTMRemoteControl vtm_remote_control{};
};

/// @brief Game Staus packet offset for comms
constexpr uint32_t REF_COMMS_GAME_STATUS_OFFSET = 0;
/// @brief Game Result packet offset for comms
constexpr uint32_t REF_COMMS_GAME_RESULT_OFFSET = 11;
/// @brief Game Robot HP packet offset for comms
constexpr uint32_t REF_COMMS_GAME_ROBOT_HP_OFFSET = 12;
/// @brief Event Data packet offset for comms
constexpr uint32_t REF_COMMS_EVENT_DATE_OFFSET = 44;
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
/// @brief End of the Ref Data packet for comms
constexpr uint32_t REF_COMMS_END_OFFSET = 136;
