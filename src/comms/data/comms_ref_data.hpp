#pragma once 

#include "comms/data/comms_data.hpp"            // for CommsData
#include <stdint.h>                             // uintN_t

struct GameStatusData {
    uint32_t competition_type = 0;
    uint32_t current_stage = 0;
    uint32_t round_time_remaining = 0;
    uint32_t _padding = 0;
    uint64_t unix_time = 0;
};

struct GameResultData {
    uint32_t winner = 0;
};

struct RobotHealthData {
    uint32_t red_hero_health = 0;
    uint32_t red_engineer_health = 0;
    uint32_t red_standard_3_health = 0;
    uint32_t red_standard_4_health = 0;
    uint32_t red_standard_5_health = 0;
    uint32_t red_sentry_health = 0;
    
    uint32_t blue_hero_health = 0;
    uint32_t blue_engineer_health = 0;
    uint32_t blue_standard_3_health = 0;
    uint32_t blue_standard_4_health = 0;
    uint32_t blue_standard_5_health = 0;
    uint32_t blue_sentry_health = 0;
};

struct GameEventData {
    uint32_t reload_zone_status = 0;
    uint32_t capture_point_status = 0;
};

struct RobotPerformanceData {
    uint32_t robot_id = 0;
    uint32_t robot_level = 0;
    uint32_t current_health = 0;
    uint32_t max_health = 0;
    uint32_t barrel_cooling_rate = 0;
    uint32_t barrel_heat_limit = 0;
    uint32_t chassis_power_limit = 0;
    uint32_t gimbal_power_active = 0;
    uint32_t chassis_power_active = 0;
    uint32_t shooter_power_active = 0;
};

struct RobotPowerHeatData {
    uint32_t chassis_voltage_output = 0;
    uint32_t chassis_current_output = 0;
    float chassis_power = 0.f;
    uint32_t buffer_energy = 0;
    uint32_t barrel_heat_1_17mm = 0;
    uint32_t barrel_heat_2_17mm = 0;
    uint32_t barrel_heat_42mm = 0;
};

struct DamageStatusData {
    uint32_t armor_plate_id = 0;
    uint32_t damage_type = 0;
};

struct LaunchingStatusData {
    uint32_t projectile_type = 0;
    uint32_t launching_mechanism = 0;
    uint32_t launching_frequency = 0;
    float initial_speed = 0.f;
};

struct ProjectileAllowanceData {
    uint32_t num_17mm = 0;
    uint32_t num_42mm = 0;
    uint32_t num_gold = 0;
};

/// @brief Ref data
struct CommsRefData : Comms::CommsData {
    CommsRefData() : CommsData(Comms::TypeLabel::CommsRefData, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(CommsRefData)) { }

    GameStatusData game_status_data;
    GameResultData game_result_data;
    RobotHealthData robot_health_data;
    GameEventData game_event_data;
    RobotPerformanceData robot_performance_data;
    RobotPowerHeatData robot_power_heat_data;
    DamageStatusData damage_status_data;
    LaunchingStatusData launching_status_data;
    ProjectileAllowanceData projectile_allowance_data;
    uint32_t _padding = 0;
};