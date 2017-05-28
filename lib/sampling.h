#pragma once

#include "geometry.h"
#include "triangle.h"
#include "xorshift.h"

#include <cmath>
#include <tuple>

namespace sampling {
namespace detail {
static __thread xorshift64star<float> uniform{4};
} // namespace detail

/**
 * Sample a point on a hemisphere.
 */
inline std::pair<Point3f, float> hemisphere() {
    // draw coordinates
    float u1 = detail::uniform();
    float u2 = detail::uniform();

    // u1 is cos(theta)
    float z = u1;
    float r = sqrtf(fmax(0.f, 1.f - z * z));
    float phi = PI2 * u2;
    float x = r * cosf(phi);
    float y = r * sinf(phi);
    return std::make_pair(Point3f{x, y, z}, u1);
}

/**
 * Sample a point on the triangle.
 *
 * @param  pos position of a corner of the triangle
 * @param  u   a side starting at pos
 * @param  v   a side (different from u) starting at pos
 * @return     point in world space
 */
inline Point3f triangle(const Point3f& pos, const Vector3f& u,
                        const Vector3f& v) {
    while (true) {
        float r1 = detail::uniform();
        float r2 = detail::uniform();

        if ((r1 + r2) <= 1.f) {
            return pos + r1 * u + r2 * v;
        }
    }
}

/**
 * Sample a point on the triangle.
 *
 * return point in world space.
 */
inline Point3f triangle(const Triangle& tri) {
    return triangle(tri.vertices[0], tri.u, tri.v);
}
} // namespace sampling
