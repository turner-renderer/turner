#pragma once

#include "triangle.h"
#include "types.h"

/**
 * Test segment and plane intersection
 *
 * Args:
 *   a, b: start and end points of the segment
 *   n, d: define the plane by the equation `n * x = d`
 *   t: out param defining the intersection point by `a + (b - a) * t`
 *
 * Return:
 *   true, if the segment intersects the plane, otherwise false
 */
inline bool intersect_segment_plane(const Point3f& a, const Point3f& b,
                                    const Normal3f& n, float d, float& t) {
    Vector3f ab = b - a;
    t = (d - dot(n, Vector3f(a))) / dot(n, ab);
    return 0.f <= t && t <= 1.f;
}

/*
 * Return min infinity if there is no intersection, otherwise return r s.t. the
 * intersection point is `ray.o + ray.d * r`.
 *
 * If n is normalized, then r is exactly the Euclidean distance from the ray to
 * the plane. Note: r may be negative.
 *
 * Args:
 *   ray: ray to intersect
 *   p0, n: Plane through p0 with normal n
 *
 * Return:
 *   true, if the ray intersects the plane, otherwise false
 *
 * Cf. http://geomalgorithms.com/a06-_intersect-2.html
 */
inline float intersect_ray_plane(const Ray& ray, const Point3f& p0,
                                 const Normal3f& n) {
    float denom = dot(n, ray.d);
    if (denom == 0.f) {
        return std::numeric_limits<float>::lowest();
    }

    float nom = dot(n, p0 - ray.o);
    return nom / denom;
}

/**
 * Intersect a ray and a triangle
 *
 * Args:
 *   ray: ray to intersect
 *   tri: triangle to intersect
 *   r: out param defining the intersection point by `ray.o + ray.d * r`
 *   s, t: baricentric coordinates on `tri` of the intersection point
 *
 * Return:
 *   true, if the ray intersects the triangle, otherwise false
 */
inline bool intersect_ray_triangle(const Ray& ray, const Triangle& tri,
                                   float& r, float& s, float& t) {
    r = intersect_ray_plane(ray, tri.vertices[0], tri.normal);
    if (r < 0) {
        return false;
    }

    Point3f P_int = ray.o + r * ray.d;
    Vector3f w = P_int - tri.vertices[0];

    // precompute scalar products
    // other values are precomputed in triangle on construction
    float wv = dot(w, tri.v);
    float wu = dot(w, tri.u);

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

/**
 * Test ray AABB (axis-aligned bounding box) intersection
 *
 * Args:
 *   ray: ray to intersect
 *   box: aabb to intersect
 *   tmin, tmax: out params defining the enter resp. exit intersection of the
 *               ray with the box by the equation `ray.o + ray.d * t`.
 *
 * Return:
 *   true, if the ray intersects the box, otherwise false
 *
 * Cf. http://people.csail.mit.edu/amy/papers/box-jgt.pdf
 */
inline bool intersect_ray_box(const Ray& ray, const Bbox3f& box, float& tmin,
                              float& tmax) {
    Vector3f d_inv(1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z);

    float tx1 = (box.p_min.x - ray.o.x) * d_inv.x;
    float tx2 = (box.p_max.x - ray.o.x) * d_inv.x;

    tmin = std::fmin(tx1, tx2);
    tmax = std::fmax(tx1, tx2);

    float ty1 = (box.p_min.y - ray.o.y) * d_inv.y;
    float ty2 = (box.p_max.y - ray.o.y) * d_inv.y;

    tmin = std::fmax(tmin, std::fmin(ty1, ty2));
    tmax = std::fmin(tmax, std::fmax(ty1, ty2));

    float tz1 = (box.p_min.z - ray.o.z) * d_inv.z;
    float tz2 = (box.p_max.z - ray.o.z) * d_inv.z;

    tmin = std::fmax(tmin, std::fmin(tz1, tz2));
    tmax = std::fmin(tmax, std::fmax(tz1, tz2));

    return !(tmax < tmin);
}

inline bool intersect_ray_box(const Ray& ray, const Bbox3f& box) {
    float tmin, tmax;
    return intersect_ray_box(ray, box, tmin, tmax);
}

/**
 * Test plane ABBB intersection
 *
 * Args:
 *   n, d: define plane by n * x = d
 *   box: box to test with
 *
 * Return:
 *   true, if intersection exists, otherwise false
 */
inline bool intersect_plane_box(const Normal3f& n, float d, const Bbox3f& box) {
    Point3f center = (box.p_max + box.p_min) * 0.5f;
    Vector3f extents = box.p_max - center;

    float r = extents.x * std::abs(n.x) + extents.y * std::abs(n.y) +
              extents.z * std::abs(n.z);

    float dist = dot(n, Vector3f(center)) - d;

    return std::abs(dist) <= r;
}

/**
 * Test triangle AABB intersection
 *
 * Implementation from
 * "Fast 3D Triangle-Bbox3f Overlap Testing"
 * by Tomas Akenine-Möller
 */
inline bool intersect_triangle_box(const Triangle& tri, const Bbox3f& box) {
    // center and extents of the box
    Point3f center = (box.p_min + box.p_max) * 0.5f;
    float e0 = (box.p_max.x - box.p_min.x) * 0.5f;
    float e1 = (box.p_max.y - box.p_min.y) * 0.5f;
    float e2 = (box.p_max.z - box.p_min.z) * 0.5f;

    // translate triangle
    Vector3f v0 = tri.vertices[0] - center;
    Vector3f v1 = tri.vertices[1] - center;
    Vector3f v2 = tri.vertices[2] - center;

    // edges of the triangle
    auto f0 = tri.u; // = v1 - v0;
    auto f1 = v2 - v1;
    auto f2 = -tri.v; // = v0 - v2;

    //
    // case 3
    //

    float p0, p1, p2, r;

    // a00 = (0,−f0z,f0y)
    p0 = v0.z * v1.y - v0.y * v1.z;
    // p0 = v0.z*f0.y - v0.y*f0.z;
    // p1 = v1.z*f0.y - v1.y*f0.z;
    p2 = v2.z * f0.y - v2.y * f0.z;
    r = e1 * std::abs(f0.z) + e2 * std::abs(f0.y);
    if (std::max(-std::max(p0, p2), std::min(p0, p2)) > r) {
        return false;
    }

    // a01 = (0,−f1z,f1y)
    p0 = v0.z * f1.y - v0.y * f1.z;
    // p1 = v1.z*f1.y - v1.z*f1.z;
    p2 = v2.z * f1.y - v2.y * f1.z;
    // p2 = v1.z*v2.y - v1.y*v2.z;
    r = e1 * std::abs(f1.z) + e2 * std::abs(f1.y);
    if (std::max(-std::max(p0, p2), std::min(p0, p2)) > r) {
        return false;
    }

    // a02 = (0,−f2z,f2y)
    // p0 = v0.y*v2.z - v0.z*v2.y;
    p0 = v0.z * f2.y - v0.y * f2.z;
    p1 = v1.z * f2.y - v1.y * f2.z;
    // p2 = v2.z*f2.y - v2.y*f2.z;
    r = e1 * std::abs(f2.z) + e2 * std::abs(f2.y);
    if (std::max(-std::max(p0, p1), std::min(p0, p1)) > r) {
        return false;
    }

    // a10 = (f0z,0,−f0x)
    p0 = v0.x * v1.z - v0.z * v1.x;
    // p1 = v0.x*f0.z - v0.z*f0.x;
    p2 = v2.x * f0.z - v2.z * f0.x;
    r = e0 * std::abs(f0.z) + e2 * std::abs(f0.x);
    if (std::max(-std::max(p0, p2), std::min(p0, p2)) > r) {
        return false;
    }

    // a11 = (f1z,0,−f1x)
    p0 = v0.x * f1.z - v0.z * f1.x;
    // p1 = v1.x*f1.z - v1.z*f1.x;
    p2 = v2.x * f1.z - v2.z * f1.x;
    r = e0 * std::abs(f1.z) + e2 * std::abs(f1.x);
    if (std::max(-std::max(p0, p2), std::min(p0, p2)) > r) {
        return false;
    }

    // a12 = (f2z,0,−f2x)
    p0 = v0.x * f2.z - v0.z * f2.x;
    p1 = v1.x * f2.z - v1.z * f2.x;
    // p2 = v2.x*f2.z - v2.z*f2.x;
    r = e0 * std::abs(f2.z) + e2 * std::abs(f2.x);
    if (std::max(-std::max(p0, p1), std::min(p0, p1)) > r) {
        return false;
    }

    // a20 = (−f0y,f0x,0)
    p0 = v0.y * f0.x - v0.x * f0.y;
    // p1 = v1.y*f0.x - v1.x*f0.y;
    p2 = v2.y * f0.x - v2.x * f0.y;
    r = e0 * std::abs(f0.y) + e1 * std::abs(f0.x);
    if (std::max(-std::max(p0, p2), std::min(p0, p2)) > r) {
        return false;
    }

    // a21 = (−f1y,f1x,0)
    p0 = v0.y * f1.x - v0.x * f1.y;
    // p1 = v1.y*f1.x - v1.x*f1.y;
    p2 = v2.y * f1.x - v2.x * f1.y;
    r = e0 * std::abs(f1.y) + e1 * std::abs(f1.x);
    if (std::max(-std::max(p0, p2), std::min(p0, p2)) > r) {
        return false;
    }

    // a22 = (−f2y,f2x,0)
    p0 = v0.y * f2.x - v0.x * f2.y;
    p1 = v1.y * f2.x - v1.x * f2.y;
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

    return intersect_plane_box(tri.normal, dot(tri.normal, v0), box);
}
