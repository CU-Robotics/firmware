#include "controller_math.hpp"

#include <algorithm>
#include <cmath>

float compute_power_limit_ratio(float buffer, float limit_thresh, float critical_thresh) {
    if (buffer >= limit_thresh) {
        return 1.0f;
    }
    return std::clamp((buffer - critical_thresh) / limit_thresh, 0.0f, 1.0f);
}

MotorVelocities xdrive_mix(float x, float y, float rot, float heading) {
    const float cosine = std::cos(heading);
    const float sine = std::sin(heading);
    return {{
        x * cosine + y * sine + rot,
        x * sine - y * cosine + rot,
        -x * cosine - y * sine + rot,
        -x * sine + y * cosine + rot,
    }};
}

float clamp1(float value) {
    return std::clamp(value, -1.0f, 1.0f);
}
