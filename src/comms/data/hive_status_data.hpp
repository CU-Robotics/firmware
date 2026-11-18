#pragma once

#include "comms/data/comms_data.hpp"    // for CommsData

/// @brief Localization state reported by Hive
enum class LocalizationStatus : uint32_t {
    Unavailable = 0,    ///< No localization is running on Hive
    Lost = 1,           ///< Localization is running but does not have a pose yet
    Localized = 2,      ///< Localization has a pose
};

/// @brief Periodic status packet from Hive, used for the status lights
struct HiveStatus : Comms::CommsData {
    /// @brief Default constructor that sets the type label, medium, priority, and size
    HiveStatus() : CommsData(Comms::TypeLabel::HiveStatus, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, sizeof(HiveStatus)) { }

    /// @brief Hive's localization state
    LocalizationStatus localization_status = LocalizationStatus::Unavailable;
};
