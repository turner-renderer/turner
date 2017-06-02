#pragma once

#include "types.h"

/**
 * Calculates Lambertian reflectance.
 *
 * Cf. https://en.wikipedia.org/wiki/Lambertian_reflectance
 *
 * @param L Unit vector towards light.
 * @param N Normal vector of surface.
 * @param C Diffuse color information of surface.
 * @param I Intensity of light.
 */
aiColor4D lambertian(const Vec& L, const Vec& N, const aiColor4D& C,
                     const aiColor4D& I) {
    float cos_alpha = std::max(0.f, dot(L, N));
    return cos_alpha * C * I;
}
