#pragma once

#include "types.h"
#include "triangle.h"
#include "intersection.h"

#include "output.h"

#define UNUSED(x) (void)(x)

/**
 * Cohen-Sutherland line clipping on AABB in 3d
 */
namespace detail {
using OutCode = int;

static constexpr int INSIDE = 0;   // 000000
static constexpr int   LEFT = 1;   // 000001
static constexpr int  RIGHT = 2;   // 000010
static constexpr int BOTTOM = 4;   // 000100
static constexpr int    TOP = 8;   // 001000
static constexpr int  FRONT = 16;  // 010000
static constexpr int   BACK = 32;  // 100000

inline OutCode compute_outcode(const Vec& v, const Box& box) {
    OutCode code = INSIDE;

    if (v.x < box.min.x) {
        code |= LEFT;
    } else if (box.max.x < v.x) {
        code |= RIGHT;
    }

    if (v.y < box.min.y) {
        code |= BOTTOM;
    } else if (box.max.y < v.y) {
        code |= TOP;
    }

    if (v.z < box.min.z) {
        code |= BACK;
    } else if (box.max.z < v.z) {
        code |= FRONT;
    }

    return code;
}
} // namespace detail

/**
 * Cohen–Sutherland clipping algorithm clips a line from `p0` to `p1` against
 * AABB `box` in 3d.
 *
 * Return:
 *   false, if the line is outside of the box, otherwise return true. In
 *   that case p0 and p1 are updated, and describe the clipped line.
 */
inline bool clip_line_aabb(Vec& p0, Vec& p1, const Box& box) {
    auto outcode_p0 = detail::compute_outcode(p0, box);
    auto outcode_p1 = detail::compute_outcode(p1, box);

    while (outcode_p0 || outcode_p1) {
        if (outcode_p0 & outcode_p1) {
            return false;
        }

        auto outcode_out = outcode_p0 ? outcode_p0 : outcode_p1;

        float x, y, z, t;
        if (outcode_out & detail::TOP) {
            t = (box.max.y - p0.y) / (p1.y - p0.y);
            x = p0.x + (p1.x - p0.x) * t;
            y = box.max.y;
            z = p0.z + (p1.z - p0.z) * t;
        } else if (outcode_out & detail::BOTTOM) {
            t = (box.min.y - p0.y) / (p1.y - p0.y);
            x = p0.x + (p1.x - p0.x) * t;
            y = box.min.y;
            z = p0.z + (p1.z - p0.z) * t;
        } else if (outcode_out & detail::RIGHT) {
            t = (box.max.x - p0.x) / (p1.x - p0.x);
            x = box.max.x;
            y = p0.y + (p1.y - p0.y) * t;
            z = p0.z + (p1.z - p0.z) * t;
        } else if (outcode_out & detail::LEFT) {
            t = (box.min.x - p0.x) / (p1.x - p0.x);
            x = box.min.x;
            y = p0.y + (p1.y - p0.y) * t;
            z = p0.z + (p1.z - p0.z) * t;
        } else if (outcode_out & detail::FRONT) {
            t = (box.max.z - p0.z) / (p1.z - p0.z);
            x = p0.x + (p1.x - p0.x) * t;
            y = p0.y + (p1.y - p0.y) * t;
            z = box.max.z;
        } else { // outcode_out & detail::BACK
            t = (box.min.z - p0.z) / (p1.z - p0.z);
            x = p0.x + (p1.x - p0.x) * t;
            y = p0.y + (p1.y - p0.y) * t;
            z = box.min.z;
        }

        if (outcode_out == outcode_p0) {
            p0.x = x;
            p0.y = y;
            p0.z = z;
            outcode_p0 = detail::compute_outcode(p0, box);
        } else {
            p1.x = x;
            p1.y = y;
            p1.z = z;
            outcode_p1 = detail::compute_outcode(p1, box);
        }
    }

    return true;
}

enum class PointPlanePos {
    ON_PLANE, BEHIND_PLANE, IN_FRONT_OF_PLANE
};

/**
 * Classify point to the thick plane given by equation `n * x = d`.
 */
inline PointPlanePos classify_point_to_plane(
    const Vec& pt, const Vec& n, float d)
{
    float dist = n * pt - d;
    if (dist > EPS) {
        return PointPlanePos::IN_FRONT_OF_PLANE;
    } else if (dist < -EPS) {
        return PointPlanePos::BEHIND_PLANE;
    }
    return PointPlanePos::ON_PLANE;
}

/**
 * Sutherland-Hodgman polygon clipping at a (thick) plane.
 */
inline std::vector<Vec>
clip_polygon_at_plane(const std::vector<Vec>& poly, const Vec& n, float d) {
    assert(poly.size() > 1);

    std::vector<Vec> points;

    Vec a = poly.back();
    auto a_side = classify_point_to_plane(a, n, d);

    for (auto b : poly) {

        auto b_side = classify_point_to_plane(b, n, d);

        if (b_side == PointPlanePos::IN_FRONT_OF_PLANE) {
            if (a_side == PointPlanePos::BEHIND_PLANE) {
                // intersect (a, b) at plane
                float t;
                bool intersects = intersect_segment_plane(a, b, n, d, t);
                UNUSED(intersects);
                assert(intersects);

                Vec pt(a + t * (b - a));
                assert(
                    classify_point_to_plane(pt, n, d)
                    == PointPlanePos::ON_PLANE);
                points.emplace_back(pt);
            }
            points.emplace_back(b);
        } else if (b_side == PointPlanePos::BEHIND_PLANE) {
            if (a_side == PointPlanePos::IN_FRONT_OF_PLANE) {
                // intesect (a, b) at plane
                float t;
                bool intersects = intersect_segment_plane(a, b, n, d, t);
                assert(intersects);
                UNUSED(intersects);

                Vec pt(a + t * (b - a));
                assert(
                    classify_point_to_plane(pt, n, d)
                    == PointPlanePos::ON_PLANE);
                points.emplace_back(pt);
            }
            // a is behind plane or on plane
        } else {
            // b is on plane
            points.emplace_back(b);
        }

        a = b;
        a_side = b_side;
    }

    return points;
}


/**
 * Clip triangle `tri` at AABB `box`.
 *
 * Requirement: triangle and box intersect.
 *
 * Return:
 *   the bounding box of the clipped polygon.
 */
inline Box clip_triangle_at_aabb(const Triangle& tri, const Box& box) {
    std::vector<Vec> points(tri.vertices.begin(), tri.vertices.end());

    // clip at 6 planes defined by box
    for (auto ax : AXES) {
        for (int side = 0; side < 2; ++side) {
            Vec normal;
            normal[ax] = side == 0 ? 1 : -1;
            float dist = side == 0 ? box.min[ax] : -box.max[ax];

            points = clip_polygon_at_plane(points, normal, dist);
            if (points.size() < 2) {
                return {};
            }
        }
    }

    // compute min and max coordinates
    Box res
        { std::numeric_limits<float>::max()
        , std::numeric_limits<float>::lowest()
        };
    for (auto ax : AXES) {
        for (const auto& pt : points) {
            if (pt[ax] < res.min[ax]) {
                res.min[ax] = pt[ax];
            }
            if (res.max[ax] < pt[ax]) {
                res.max[ax] = pt[ax];
            }
        }
    }

    return res;
}
