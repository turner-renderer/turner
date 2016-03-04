#include "triangle.h"
#include "types.h"

//
// Helper functions for triangle aabb intersection
//

bool axistest(
    const Vec& e,
    const Vec& v0, const Vec& v1,
    const Axis ax0, const Axis ax1,
    const Vec& box_halfsize)
{
    auto p0 = e[ax1] * v0[ax0] - e[ax0] * v0[ax1];
    auto p2 = e[ax1] * v1[ax0] - e[ax0] * v1[ax1];
    float min, max;
    if (p0 < p2) {
        min = p0;
        max = p2;
    } else {
        min = p2;
        max = p0;
    }

    auto rad = std::abs(e[ax0]) * box_halfsize[ax0]
        + std::abs(e[ax1]) * box_halfsize[ax1];
    return !(min > rad || max < -rad);
}

//
// Compute min and max of 3 float values.
// The min value is then in a, and the max value in c.
//
void minmax(float& a, float& b, float& c) {
    if (a < b) {
        if (b < c) {
            return;
        } else {
            std::swap(b, c);
        }
    } else {
        std::swap(a, b);
        if (b < c) {
            return;
        } else {
            std::swap(b, c);
            if (a < b) {
                return;
            } else {
                std::swap(a, b);
            }
        }
    }
}

bool Triangle::axis_intersection(const Axis ax, const Vec& box_halfsize) const
{
    auto min = vertices[0][ax];
    auto mid = vertices[1][ax];
    auto max = vertices[2][ax];
    minmax(min, mid, max);
    return !(min > box_halfsize[ax]) && !(max < -box_halfsize[ax]);
}

bool plane_box_intersect(const Vec& normal, const Vec& v, const Vec& max_box)
{
    Vec vmin, vmax;
    Axis ax = Axis::X;
    for (int i = 0; i < 3; ++i, ++ax) {
        auto u = v[ax];
        if (normal[ax] > 0.f) {
            vmin[ax] = - max_box[ax] - u;
            vmax[ax] = max_box[ax] - u;
        } else {
            vmin[ax] = max_box[ax] - u;
            vmax[ax] = - max_box[ax] - u;
        }
    }

    if (normal * vmin > 0.f) {
        return false;
    }
    if (normal * vmax >= 0.f) {
        return true;
    }

    return false;
}

//
// Triangle AABB intersection test
//
// Implementation
// "Fast 3D Triangle-Box Overlap Testing"
// by Tomas Akenine-Möller
//
bool Triangle::intersect(const Box& box) const {
    Vec box_halfsize = (box.max - box.min) / 2.f;
    Vec box_center = box.min + box_halfsize;

    // translation box center -> 0
    auto v0 = vertices[0] - box_center;
    auto v1 = vertices[1] - box_center;
    auto v2 = vertices[2] - box_center;

    // triangle edges
    auto e0 = u;       // == v1 - v0
    auto e1 = v2 - v1; // == v2 - v0
    auto e2 = -v;      // == v0 - v2

    // 9 axis tests
    if (!axistest(e0, v0, v2, Axis::Y, Axis::Z, box_halfsize) ||
        !axistest(e0, v0, v2, Axis::Z, Axis::X, box_halfsize) ||
        !axistest(e0, v1, v2, Axis::X, Axis::Y, box_halfsize) ||
        !axistest(e1, v0, v2, Axis::Y, Axis::Z, box_halfsize) ||
        !axistest(e1, v0, v2, Axis::Z, Axis::X, box_halfsize) ||
        !axistest(e1, v0, v1, Axis::X, Axis::Y, box_halfsize) ||
        !axistest(e2, v0, v1, Axis::Y, Axis::Z, box_halfsize) ||
        !axistest(e2, v0, v1, Axis::Z, Axis::X, box_halfsize) ||
        !axistest(e2, v1, v2, Axis::X, Axis::Z, box_halfsize))
    {
        return false;
    }

    if (!axis_intersection(Axis::X, box_halfsize) ||
        !axis_intersection(Axis::Y, box_halfsize) ||
        !axis_intersection(Axis::Z, box_halfsize))
    {
        return false;
    }

    // test if box intersects the plane defined by the triangle
    return plane_box_intersect(normal, v0, box_halfsize);
}

