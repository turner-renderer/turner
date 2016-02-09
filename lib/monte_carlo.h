#pragma once

#include "types.h"
#include "xorshift.h"

#include <cmath>

class Hemisphere {
public:
    Hemisphere() : uniform(4) {}

    const Vec sample() {
        // Draw polar coordinates
        const float theta = invertedCDFtheta(uniform());
        const float phi = invertedCDFphi(uniform());

        // Polar to cartesian coordinates
        return Vec(cos(phi)*cos(theta), sin(theta), sin(phi)*cos(phi));
    }

private:
    xorshift64star<float> uniform;

    /**
     * Sample theta from its inverted CDF.
     *
     * @param y Uniform random variable ∈ [0.0, 1.0]
     */
    const float invertedCDFtheta(const float y) {
        return acos(1.f - y);
    }

    /**
     * Sample phi from its inverted CDF.
     *
     * @param y Uniform random variable ∈ [0.0, 1.0]
     */
    const float invertedCDFphi(const float y) {
        return y * 2.f * M_PI;
    }
};
