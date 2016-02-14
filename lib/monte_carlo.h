#pragma once

#include "types.h"
#include "xorshift.h"

#include <cmath>


namespace {
    static constexpr float M_2PI = 2.f * M_PI;
}


class Hemisphere {
public:
    // TODO: Docs
    std::pair<Vec, float> sample() {
        // draw coordinates
        float u1 = uniform();
        float u2 = uniform();

        // u1 is cos(theta)
        auto z = u1;
        auto r = sqrtf(fmax(0.f, 1.f - z*z));
        auto phi = M_2PI * u2;
        auto x = r * cosf(phi);
        auto y = r * sinf(phi);
        return std::make_pair(Vec{x, y, z}, u1);
    }

private:
    // TODO: Use seed and make sure it is thread safe
    xorshift64star<float> uniform{4};
};
