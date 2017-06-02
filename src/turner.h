#pragma once

#include <cmath>
#include <limits>
#include <ostream>

namespace turner {

#define UNUSED(x) (void)(x)

//
// Constants
//

#undef INFINITY

static constexpr float PI = static_cast<float>(M_PI);
static constexpr float PI2 = static_cast<float>(2 * M_PI);
static constexpr float INFINITY = std::numeric_limits<float>::max();
// TODO: Check how to define fat planes properly.
static constexpr float EPS = 0.00001f;

inline bool is_eps_zero(float a) { return std::abs(a) < EPS; }

inline float lerp(float t, float a, float b) { return (1 - t) * a + t * b; }

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

/**
 * Transform sRGB value to non-linear sR'G'B'.
 *
 * Cf. https://www.w3.org/Graphics/Color/sRGB
 *
 * @param  value color value to transform
 * @return       transformed value
 */
inline float gamma_correction(float value) {
    if (value <= 0.0031308f) {
        return 12.92f * value;
    }
    return 1.055f * std::pow(value, 1.f / 2.4f) - 0.055f;
}

/**
 * Generic output operator
 */
template <typename T>
using has_to_string =
    typename std::is_same<decltype(std::declval<const T>().to_string()),
                          std::string>::type;

template <typename T, typename = std::enable_if<has_to_string<T>::value>>
std::ostream& operator<<(std::ostream& out, const T& val) {
    return out << val.to_string();
}
} // namespace turner
