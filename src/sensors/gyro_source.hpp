#pragma once
#include "source.hpp"

/// @brief Interface for any gyro that provides angular velocity readings.
class GyroSource {
  public:
    /// @brief Use derived destructor.
    virtual ~GyroSource() = default;
    /// @brief Get angular velocity of the gyro arounds the X axis.
    /// @return Angular velocity around the X axis.
    virtual float get_gyro_X() = 0;
    /// @brief Get angular velocity of the gyro arounds the Y axis.
    /// @return Angular velocity around the Y axis.
    virtual float get_gyro_Y() = 0;
    /// @brief Get angular velocity of the gyro arounds the Z axis.
    /// @return Angular velocity around the Z axis.
    virtual float get_gyro_Z() = 0;
};

template <> inline constexpr bool is_source_v<GyroSource> = true;
