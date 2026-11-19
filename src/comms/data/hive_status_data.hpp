#pragma once

#include "comms/data/comms_data.hpp"    // for CommsData

/// @brief Localization state from Hive. Must match hive-rs.
enum class LocalizationStatus : uint32_t {
    Unavailable = 0,    ///< No localization running
    Lost = 1,           ///< Running, no pose yet
    Localized = 2,      ///< Has a pose
};

/// @brief Status packet from Hive. Must match hive-rs.
struct HiveStatus : Comms::CommsData {
    /// @brief Default constructor
    HiveStatus() : CommsData(Comms::TypeLabel::HiveStatus, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(HiveStatus)) { }

    /// @brief Localization state
    LocalizationStatus localization_status = LocalizationStatus::Unavailable;
};
