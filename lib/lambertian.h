#pragma once

#include "geometry.h"
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
Color lambertian(const Vector3f& L, const Normal3f& N, const Color& C,
                 const Color& I) {
    float cos_alpha = std::max(0.f, dot(L, N));
    return cos_alpha * C * I;
}
