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
};

template <> inline constexpr bool is_source_v<EncoderSource> = true;