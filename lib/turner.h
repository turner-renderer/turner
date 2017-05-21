#pragma once

#include <cmath>
#include <limits>

namespace turner {

#undef INFINITY

constexpr float PI = static_cast<float>(M_PI);
constexpr float INFINITY = std::numeric_limits<float>::max();

float lerp(float t, float a, float b) { return (1 - t) * a + t * b; }

template <typename T, typename U, typename V>
inline T clamp(T val, U low, V high) {
    if (val < low) {
        return low;
    } else if (val > high) {
        return high;
    } else {
        return val;
    }
}

// TODO: Docs and explanation of values.
inline float gamma_correction(float value) {
    if (value <= 0.0031308f) {
        return 12.92f * value;
    }
    return 1.055f * std::pow(value, 1.f / 2.4f) - 0.055f;
}

} // namespace turner
