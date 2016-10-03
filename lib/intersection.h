#pragma once

#include "triangle.h"
#include "types.h"

//
// Test segment and plane intersection
//
// Args:
//   a, b: start and end points of the segment
//   n, d: define the plane by the equation `n * x = d`
//   t: out param defining the intersection point by `a + (b - a) * t`
//
// Return:
//   true, if the segment intersects the plane, otherwise false
//
inline bool intersect_segment_plane(
    const Vec& a, const Vec& b, const Vec& n, float d, float& t)
{
    auto ab = b - a;
    t = (d - n * a) / (n * ab);
    return 0.f <= t && t <= 1.f;
}

//
// Return min infinity if there is no intersection, otherwise return r s.t. the
// intersection point is `ray.pos + ray.dir * r`.
//
// If n is normalized, then r is exactly the Euclidean distance from the ray to
// the plane. Note: r may be negative.
//
// Args:
//   ray: ray to intersect
//   v0, n: Plane through v0 with normal n
//
// Return:
//   true, if the ray intersects the plane, otherwise false
//
// Cf. http://geomalgorithms.com/a06-_intersect-2.html
//
inline float intersect_ray_plane(const Ray& ray, const Vec& v0, const Vec& n) {
    auto denom = n * ray.dir;
    if (denom == 0.f) {
        return std::numeric_limits<float>::lowest();
    }

    auto nom = n * (v0 - ray.pos);
    return nom / denom;
}


//
// Intersect a ray and a triangle
//
// Args:
//   ray: ray to intersect
//   tri: triangle to intersect
//   r: out param defining the intersection point by `ray.pos + ray.dir * r`
//   s, t: baricentric coordinates on `tri` of the intersection point
//
// Return:
//   true, if the ray intersects the triangle, otherwise false
//
inline bool intersect_ray_triangle(const Ray& ray, const Triangle& tri,
    float& r, float& s, float& t)
{
    r = intersect_ray_plane(ray, tri.vertices[0], tri.normal);
    if (r < 0) {
        return false;
    }

    auto P_int = ray.pos + r * ray.dir;
    auto w = P_int - tri.vertices[0];

    // precompute scalar products
    // other values are precomputed in triangle on construction
    auto wv = w*tri.v;
    auto wu = w*tri.u;

    s = (tri.uv * wv - tri.vv * wu) / tri.denom;
    if (s < 0) {
        return false;
    }

    t = (tri.uv * wu - tri.uu * wv) / tri.denom;
    if (t < 0 || 1 < s + t) {
        return false;
    }

    return true;
}


//
// Test ray AABB (axis-aligned bounding box) intersection
//
// Args:
//   ray: ray to intersect
//   box: aabb to intersect
//   tmin, tmax: out params defining the enter resp. exit intersection of the
//     halbray with the box by the equation `ray.pos + ray.dir * t`.
//
// Return:
//   true, if the ray intersects the box, otherwise false
//
// Cf. http://people.csail.mit.edu/amy/papers/box-jgt.pdf
//
inline bool intersect_ray_box(
    const Ray& ray, const Box& box, float& tmin, float& tmax)
{
    float tx1 = (box.min.x - ray.pos.x) * ray.invdir.x;
    float tx2 = (box.max.x - ray.pos.x) * ray.invdir.x;

    tmin = std::fmin(tx1, tx2);
    tmax = std::fmax(tx1, tx2);

    float ty1 = (box.min.y - ray.pos.y) * ray.invdir.y;
    float ty2 = (box.max.y - ray.pos.y) * ray.invdir.y;

    tmin = std::fmax(tmin, std::fmin(ty1, ty2));
    tmax = std::fmin(tmax, std::fmax(ty1, ty2));

    float tz1 = (box.min.z - ray.pos.z) * ray.invdir.z;
    float tz2 = (box.max.z - ray.pos.z) * ray.invdir.z;

    tmin = std::fmax(tmin, std::fmin(tz1, tz2));
    tmax = std::fmin(tmax, std::fmax(tz1, tz2));

    return !(tmax < tmin);
}

inline bool intersect_ray_box(const Ray& ray, const Box& box) {
    float tmin, tmax;
    return intersect_ray_box(ray, box, tmin, tmax);
}


//
// Test plane ABBB intersection
//
// Args:
//   n, d: define plane by n * x = d
//   box: box to test with
//
// Return:
//   true, if intersection exists, otherwise false
//
inline bool intersect_plane_box(const Vec& n, float d, const Box& box) {
    auto center = (box.max + box.min) * 0.5f;
    auto extents = box.max - center;

    float r =
        extents.x * std::abs(n.x) +
        extents.y * std::abs(n.y) +
        extents.z * std::abs(n.z);

    float dist = n * center - d;

    return std::abs(dist) <= r;
}


//
// Test triangle AABB intersection
//
// Implementation from
// "Fast 3D Triangle-Box Overlap Testing"
// by Tomas Akenine-Möller
//
inline bool intersect_triangle_box(const Triangle& tri, const Box& box) {
    // center and extents of the box
    auto center = (box.min + box.max) * 0.5f;
    float e0 = (box.max.x - box.min.x) * 0.5f;
    float e1 = (box.max.y - box.min.y) * 0.5f;
    float e2 = (box.max.z - box.min.z) * 0.5f;

    // translate triangle
    auto v0 = tri.vertices[0] - center;
    auto v1 = tri.vertices[1] - center;
    auto v2 = tri.vertices[2] - center;

    // edges of the triangle
    auto f0 = tri.u;   // = v1 - v0;
    auto f1 = v2 - v1;
    auto f2 = -tri.v;  // = v0 - v2;

    //
    // case 3
    //

    float p0, p1, p2, r;

    // a00 = (0,−f0z,f0y)
    p0 = v0.z*v1.y - v0.y*v1.z;
    // p0 = v0.z*f0.y - v0.y*f0.z;
    // p1 = v1.z*f0.y - v1.y*f0.z;
    p2 = v2.z*f0.y - v2.y*f0.z;
    r = e1 * std::abs(f0.z) + e2 * std::abs(f0.y);
    if (std::max(-std::max(p0, p2), std::min(p0, p2)) > r) {
        return false;
    }


    // a01 = (0,−f1z,f1y)
    p0 = v0.z*f1.y - v0.y*f1.z;
    // p1 = v1.z*f1.y - v1.z*f1.z;
    p2 = v2.z*f1.y - v2.y*f1.z;
    // p2 = v1.z*v2.y - v1.y*v2.z;
    r = e1 * std::abs(f1.z) + e2 * std::abs(f1.y);
    if (std::max(-std::max(p0, p2), std::min(p0, p2)) > r) {
        return false;
    }

    // a02 = (0,−f2z,f2y)
    // p0 = v0.y*v2.z - v0.z*v2.y;
    p0 = v0.z*f2.y - v0.y*f2.z;
    p1 = v1.z*f2.y - v1.y*f2.z;
    // p2 = v2.z*f2.y - v2.y*f2.z;
    r = e1 * std::abs(f2.z) + e2 * std::abs(f2.y);
    if (std::max(-std::max(p0, p1), std::min(p0, p1)) > r) {
        return false;
    }

    // a10 = (f0z,0,−f0x)
    p0 = v0.x*v1.z - v0.z*v1.x;
    // p1 = v0.x*f0.z - v0.z*f0.x;
    p2 = v2.x*f0.z - v2.z*f0.x;
    r = e0 * std::abs(f0.z) + e2 * std::abs(f0.x);
    if (std::max(-std::max(p0, p2), std::min(p0, p2)) > r) {
        return false;
    }

    // a11 = (f1z,0,−f1x)
    p0 = v0.x*f1.z - v0.z*f1.x;
    // p1 = v1.x*f1.z - v1.z*f1.x;
    p2 = v2.x*f1.z - v2.z*f1.x;
    r = e0 * std::abs(f1.z) + e2 * std::abs(f1.x);
    if (std::max(-std::max(p0, p2), std::min(p0, p2)) > r) {
        return false;
    }

    // a12 = (f2z,0,−f2x)
    p0 = v0.x*f2.z - v0.z*f2.x;
    p1 = v1.x*f2.z - v1.z*f2.x;
    // p2 = v2.x*f2.z - v2.z*f2.x;
    r = e0 * std::abs(f2.z) + e2 * std::abs(f2.x);
    if (std::max(-std::max(p0, p1), std::min(p0, p1)) > r) {
        return false;
    }

    // a20 = (−f0y,f0x,0)
    p0 = v0.y*f0.x - v0.x*f0.y;
    // p1 = v1.y*f0.x - v1.x*f0.y;
    p2 = v2.y*f0.x - v2.x*f0.y;
    r = e0 * std::abs(f0.y) + e1 * std::abs(f0.x);
    if (std::max(-std::max(p0, p2), std::min(p0, p2)) > r) {
        return false;
    }

    // a21 = (−f1y,f1x,0)
    p0 = v0.y*f1.x - v0.x*f1.y;
    // p1 = v1.y*f1.x - v1.x*f1.y;
    p2 = v2.y*f1.x - v2.x*f1.y;
    r = e0 * std::abs(f1.y) + e1 * std::abs(f1.x);
    if (std::max(-std::max(p0, p2), std::min(p0, p2)) > r) {
        return false;
    }

    // a22 = (−f2y,f2x,0)
    p0 = v0.y*f2.x - v0.x*f2.y;
    p1 = v1.y*f2.x - v1.x*f2.y;
    // p2 = v2.y*f2.x - v2.x*f2.y;
    r = e0 * std::abs(f2.y) + e1 * std::abs(f2.x);
    if (std::max(-std::max(p0, p1), std::min(p0, p1)) > r) {
        return false;
    }

    //
    // Case 1
    //

    if (e0 < min(v0.x, v1.x, v2.x) || max(v0.x, v1.x, v2.x) < -e0) {
        return false;
    }
    if (e1 < min(v0.y, v1.y, v2.y) || max(v0.y, v1.y, v2.y) < -e1) {
        return false;
    }
    if (e2 < min(v0.z, v1.z, v2.z) || max(v0.z, v1.z, v2.z) < -e2) {
        return false;
    }

    //
    // Case 2
    //

    // return plane_box_intersect(tri.normal, v0, Vec({e0, e1, e2}));
    return intersect_plane_box(tri.normal, tri.normal * v0, box);
}
