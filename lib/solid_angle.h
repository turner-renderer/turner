#pragma once

#include <cmath>
#include "triangle.h"
#include "types.h"

/**
 * Returns the solid angle subtended by tirangle.
 *
 * See Eriksson, Folke (1990). "On the measure of solid angles". Math. Mag. 63
 * (3): 184–187.
 */
float solid_angle(const Vec& O, const Triangle& triangle) {

    // Project triangle vetrices onto unit sphere around O
    Vec a = (triangle.vertices[0] - O).Normalize();
    Vec b = (triangle.vertices[1] - O).Normalize();
    Vec c = (triangle.vertices[2] - O).Normalize();

    float tan_omega_2 = (a * (b ^ c)) / ( 1 +  b*c + c*a + a*b);

    return atan(tan_omega_2) * 2;
}
