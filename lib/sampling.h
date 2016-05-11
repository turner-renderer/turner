#pragma once

#include "triangle.h"
#include "types.h"
#include "xorshift.h"

#include <cmath>
#include <tuple>

namespace {
    static constexpr float M_2PI = 2.f * M_PI;
}

namespace sampling {

    xorshift64star<float> uniform{4};

    std::pair<Vec, float> hemisphere() {
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

    Vec triangle(const Triangle& triangle) {
        while(true) {
            float r1 = uniform();
            float r2 = uniform();

            if ((r1 + r2) <= 1.f) {
                return triangle.vertices[0] + r1 * triangle.u + r2 * triangle.v;
            }
        }
    }
}

