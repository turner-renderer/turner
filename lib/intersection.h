#pragma once

#include "types.h"

inline bool intersect_segment_plane(
    const Vec& a, const Vec& b, const Vec& n, float d, float& t)
{
    auto ab = b - a;
    t = (d - n * a) / (n * ab);
    if (0.f <= t && t <= 1.f) {
        return true;
    }
    return false;
}

//
// Return min infinity if there is no intersection, otherwise return r s.t. the
// intersection point is P0 + r * (P1 - P0).
//
// If n is normalized, then r is exactly the Euclidean distance from the ray to
// the plane. Note: r may be negative.
//
// Args:
//   v0, n: Plane through v0 with normal n
//
// Cf. http://geomalgorithms.com/a06-_intersect-2.html
//
inline float ray_plane_intersection(const Ray& ray, const Vec& v0, const Vec& n) {
    auto denom = n * ray.dir;
    if (denom == 0.f) {
        return std::numeric_limits<float>::lowest();
    }

    auto nom = n * (v0 - ray.pos);
    return nom / denom;
}

//
// Intersect a ray with an AABB (axis-aligned bounding box).
//
// Cf. http://people.csail.mit.edu/amy/papers/box-jgt.pdf
//
inline bool ray_box_intersection(const Ray& r, const Box& box) {
    float tx1 = (box.min.x - r.pos.x) * r.invdir.x;
    float tx2 = (box.max.x - r.pos.x) * r.invdir.x;

    float tmin = std::fmin(tx1, tx2);
    float tmax = std::fmax(tx1, tx2);

    float ty1 = (box.min.y - r.pos.y) * r.invdir.y;
    float ty2 = (box.max.y - r.pos.y) * r.invdir.y;

    tmin = std::fmax(tmin, std::fmin(ty1, ty2));
    tmax = std::fmin(tmax, std::fmax(ty1, ty2));

    float tz1 = (box.min.z - r.pos.z) * r.invdir.z;
    float tz2 = (box.max.z - r.pos.z) * r.invdir.z;

    tmin = std::fmax(tmin, std::fmin(tz1, tz2));
    tmax = std::fmin(tmax, std::fmax(tz1, tz2));

    return !(tmax < tmin);
}
