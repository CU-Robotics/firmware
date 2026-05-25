#pragma once

#include "comms/data/comms_data.hpp" // for CommsData
#include <stdint.h>                  // uintN_t

/// @copydoc GameStatus
struct GameStatusData {
    /// @copydoc GameStatus::competition_type
    uint32_t competition_type = 0;
    /// @copydoc GameStatus::current_stage
    uint32_t current_stage = 0;
    /// @copydoc GameStatus::round_time_remaining
    uint32_t round_time_remaining = 0;
    /// @brief padding to align with uint64_t
    uint32_t _padding = 0;
    /// @copydoc GameStatus::unix_time
    uint64_t unix_time = 0;
};
/// @copydoc GameResult
struct GameResultData {
    /// @copydoc GameResult::winner
    uint32_t winner = 0;
};

/// @copydoc GameRobotHP
struct RobotHealthData {
    /// @copydoc GameRobotHP::hero_health
    uint32_t hero_health = 0;
    /// @copydoc GameRobotHP::engineer_health
    uint32_t engineer_health = 0;
    /// @copydoc GameRobotHP::standard_3_health
    uint32_t standard_3_health = 0;
    /// @copydoc GameRobotHP::standard_4_health
    uint32_t standard_4_health = 0;
    /// @copydoc GameRobotHP::sentry_health
    uint32_t sentry_health = 0;
};
/// @copydoc EventData
struct GameEventData {
    /// @copydoc EventData::reload_zone_status
    uint32_t reload_zone_status = 0;
    /// @copydoc EventData::capture_point_status
    uint32_t capture_point_status = 0;
};
/// @copydoc RobotPerformance
struct RobotPerformanceData {
    /// @copydoc RobotPerformance::robot_ID
    uint32_t robot_id = 0;
    /// @copydoc RobotPerformance::robot_level
    uint32_t robot_level = 0;
    /// @copydoc RobotPerformance::current_HP
    uint32_t current_health = 0;
    /// @copydoc RobotPerformance::max_HP
    uint32_t max_health = 0;
    /// @copydoc RobotPerformance::barrel_cooling_rate
    uint32_t barrel_cooling_rate = 0;
    /// @copydoc RobotPerformance::barrel_heat_limit
    uint32_t barrel_heat_limit = 0;
    /// @copydoc RobotPerformance::chassis_power_limit
    uint32_t chassis_power_limit = 0;
    /// @copydoc RobotPerformance::gimbal_power_active
    uint32_t gimbal_power_active = 0;
    /// @copydoc RobotPerformance::chassis_power_active
    uint32_t chassis_power_active = 0;
    /// @copydoc RobotPerformance::shooter_power_active
    uint32_t shooter_power_active = 0;
};
/// @copydoc RobotPowerHeat
struct RobotPowerHeatData {
    /// @copydoc RobotPowerHeat::buffer_energy
    uint32_t buffer_energy = 0;
    /// @copydoc RobotPowerHeat::barrel_heat_1_17mm
    uint32_t barrel_heat_1_17mm = 0;
    /// @copydoc RobotPowerHeat::barrel_heat_42mm
    uint32_t barrel_heat_42mm = 0;
};

/// @copydoc DamageStatus
struct DamageStatusData {
    /// @copydoc DamageStatus::armor_plate_ID
    uint32_t armor_plate_ID = 0;
    /// @copydoc DamageStatus::damage_type
    uint32_t damage_type = 0;
};

/// @copydoc LaunchingStatus
struct LaunchingStatusData {
    /// @copydoc LaunchingStatus::projectile_type
    uint32_t projectile_type = 0;
    /// @copydoc LaunchingStatus::launching_mechanism
    uint32_t launching_mechanism = 0;
    /// @copydoc LaunchingStatus::launching_frequency
    uint32_t launching_frequency = 0;
    /// @copydoc LaunchingStatus::initial_speed
    float initial_speed = 0.f;
};
/// @copydoc ProjectileAllowance
struct ProjectileAllowanceData {
    /// @copydoc ProjectileAllowance::num_17mm
    uint32_t num_17mm = 0;
    /// @copydoc ProjectileAllowance::num_42mm
    uint32_t num_42mm = 0;
    /// @copydoc ProjectileAllowance::num_gold
    uint32_t num_gold = 0;
};

/// @brief Mega struct fo al of the ref data we want to send over comms
struct CommsRefData : Comms::CommsData {
    /// @brief default constructor that initializes the CommsData base class with the appropriate type label, physical medium, priority, and size for this struct
    CommsRefData() : CommsData(Comms::TypeLabel::CommsRefData, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(CommsRefData)) {}
    /// @brief status of the current game
    GameStatusData game_status_data;
    /// @brief result of the current game
    GameResultData game_result_data;
    /// @brief health of all of the robots in the game
    RobotHealthData robot_health_data;
    /// @brief status of the game events such as reload zones and capture points
    GameEventData game_event_data;
    /// @brief performance data for our robot such as current HP, power limits, and cooling rates
    RobotPerformanceData robot_performance_data;
    /// @brief real-time power and heat data for our robot
    RobotPowerHeatData robot_power_heat_data;
    /// @brief damage status data for our robot
    DamageStatusData damage_status_data;
    /// @brief launching status data for our robot
    LaunchingStatusData launching_status_data;
    /// @brief projectile allowance data for our robot
    ProjectileAllowanceData projectile_allowance_data;
    /// @brief padding to ensure proper alignment
    uint32_t _padding = 0;
};
