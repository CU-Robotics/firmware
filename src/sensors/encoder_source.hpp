#pragma once
#include "source.hpp"

/// @brief Interface for any encoder that provides angular value.
class EncoderSource {
  public:
    /// @brief Use derived destructor.
    virtual ~EncoderSource() = default;
    /// @brief Get angle of the last read function.
    /// @return Read angle (radians).
    virtual float get_angle() const = 0;
    /// @brief Whether at least one valid reading has been received.
    /// @return true once get_angle() reflects real sensor data.
    virtual bool has_valid_reading() const { return true; }
};

template <> inline constexpr bool is_source_v<EncoderSource> = true;