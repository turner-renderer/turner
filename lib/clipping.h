#include "types.h"
#include "triangle.h"

//
// Cohen-Sutherland line clipping on AABB in 3d
//

// TODO: Remove

using OutCode = int;

static constexpr int INSIDE = 0;   // 000000
static constexpr int   LEFT = 1;   // 000001
static constexpr int  RIGHT = 2;   // 000010
static constexpr int BOTTOM = 4;   // 000100
static constexpr int    TOP = 8;   // 001000
static constexpr int  FRONT = 16;  // 010000
static constexpr int   BACK = 32;  // 100000

OutCode compute_outcode(const Vec& v, const Box& box) {
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

//
// Cohen–Sutherland clipping algorithm clips a line from `p0` to `p1` against
// AABB `box` in 3d.
//
// Return:
//   false, if the line is outside of the box, otherwise return true. In
//   that case p0 and p1 are updated, and describe the clipped line.
//
bool clip_line_aabb(Vec& p0, Vec& p1, const Box& box) {
    auto outcode0 = compute_outcode(p0, box);
    auto outcode1 = compute_outcode(p1, box);

    while (outcode0 || outcode1) {
        if (outcode0 & outcode1) {
            return false;
        }

        auto outcode_out = outcode0 ? outcode0 : outcode1;

        float x, y, z, t;
        if (outcode_out & TOP) {
            t = (box.max.y - p0.y) / (p1.y - p0.y);
            x = p0.x + (p1.x - p0.x) * t;
            y = box.max.y;
            z = p0.z + (p1.z - p0.z) * t;
        } else if (outcode_out & BOTTOM) {
            t = (box.min.y - p0.y) / (p1.y - p0.y);
            x = p0.x + (p1.x - p0.x) * t;
            y = box.min.y;
            z = p0.z + (p1.z - p0.z) * t;
        } else if (outcode_out & RIGHT) {
            t = (box.max.x - p0.x) / (p1.x - p0.x);
            x = box.max.x;
            y = p0.y + (p1.y - p0.y) * t;
            z = p0.z + (p1.z - p0.z) * t;
        } else if (outcode_out & LEFT) {
            t = (box.min.x - p0.x) / (p1.x - p0.x);
            x = box.min.x;
            y = p0.y + (p1.y - p0.y) * t;
            z = p0.z + (p1.z - p0.z) * t;
        } else if (outcode_out & FRONT) {
            t = (box.max.z - p0.z) / (p1.z - p0.z);
            x = p0.x + (p1.x - p0.x) * t;
            y = p0.y + (p1.y - p0.y) * t;
            z = box.max.z;
        } else if (outcode_out & BACK) {
            t = (box.min.z - p0.z) / (p1.z - p0.z);
            x = p0.x + (p1.x - p0.x) * t;
            y = p0.y + (p1.y - p0.y) * t;
            z = box.min.z;
        }

        if (outcode_out == outcode0) {
            p0.x = x;
            p0.y = y;
            p0.z = z;
            outcode0 = compute_outcode(p0, box);
        } else {
            p1.x = x;
            p1.y = y;
            p1.z = z;
            outcode1 = compute_outcode(p1, box);
        }
    }

    return true;
}


//
// Clip triangle `tri` at AABB `box`.
//
// Return:
//   the bounding box of the clipped polynomial. If the triangle is outside of
//   the box, zero box is return.
//
Box triangle_clip_aabb(const Triangle& tri, const Box& box) {
    Vec min, max;
    auto tri_box = tri.bbox();

    min.x = std::max(tri_box.min.x, box.min.x);
    min.y = std::max(tri_box.min.y, box.min.y);
    min.z = std::max(tri_box.min.z, box.min.z);

    max.x = std::min(tri_box.max.x, box.max.x);
    max.y = std::min(tri_box.max.y, box.max.y);
    max.z = std::min(tri_box.max.z, box.max.z);

    return min <= max ? Box{min, max} : Box{};
}
