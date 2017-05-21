#pragma once

#include <cmath>
#include <limits>

namespace turner {

constexpr float PI = static_cast<float>(M_PI);
constexpr float INFINITY = std::numeric_limits<float>::max();

float lerp(float t, float a, float b) { return (1 - t) * a + t * b; }

} // namespace turner
