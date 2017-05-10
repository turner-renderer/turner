#include <limits>

#pragma once

namespace turner {

constexpr float INFINITY = std::numeric_limits<float>::max();

float lerp(float t, float a, float b) { return (1 - t) * a + t * b; }

} // namespace turner
