#pragma once

#include "triangle.h"
#include "types.h"
#include "xorshift.h"

#include <cmath>
#include <tuple>

namespace sampling {

/**
 * Sample a point on a hemisphere.
 */
std::pair<Vec, float> hemisphere();

/**
 * Sample a point on the triangle.
 *
 * return point in world space.
 */
Vec triangle(const Triangle& triangle);

}

