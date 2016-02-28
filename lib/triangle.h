#pragma once

#include "types.h"
#include "intersection.h"
#include <vector>
#include <array>

//
// Be aware of modifying data in the triangle after its construction. The
// precomputed values won't be updated.
//
class Triangle {
public:
    Triangle(
        std::array<Vec, 3> vs,
        std::array<Vec, 3> ns,
        const aiColor4D& ambient,
        const aiColor4D& diffuse)
    : vertices(vs)
    , normals(ns)
    , ambient(ambient)
    , diffuse(diffuse)
    , u(vertices[1] - vertices[0])
    , v(vertices[2] - vertices[0])
    , normal(u^v)
    , uv(u*v)
    , vv(v*v)
    , uu(u*u)
    , denom(uv * uv - uu * vv)
    {}

    bool intersect(const Ray& ray, float& r, float& s, float& t) const {
        r = ray_plane_intersection(ray, vertices[0], normal);
        if (r < 0) {
            return false;
        }

        auto P_int = ray.pos + r * ray.dir;
        auto w = P_int - vertices[0];

        // precompute scalar products
        auto wv = w*v;
        auto wu = w*u;

        s = (uv * wv - vv * wu) / denom;
        if (s < 0) {
            return false;
        }

        t = (uv * wu - uu * wv) / denom;
        if (t < 0 || s + t > 1) {
            return false;
        }

        return true;
    }

    // Triangle AABB intersection test
    bool intersect(const Box& box) const;

    //
    // Interpolate normal using barycentric coordinates.
    //
    // Requirement: r + s + t == 1
    //
    Vec interpolate_normal(float r, float s, float t) const {
        auto normal = r * normals[0] + s * normals[1] + t * normals[2];
        normal.Normalize();
        return normal;
    }


    // bounding box of triangle
    Box bbox() const {
        Vec min =
            { fmin(vertices[0].x, vertices[1].x, vertices[2].x)
            , fmin(vertices[0].y, vertices[1].y, vertices[2].y)
            , fmin(vertices[0].z, vertices[1].z, vertices[2].z)
            };

        Vec max =
            { fmax(vertices[0].x, vertices[1].x, vertices[2].x)
            , fmax(vertices[0].y, vertices[1].y, vertices[2].y)
            , fmax(vertices[0].z, vertices[1].z, vertices[2].z)
            };

        return {min, max};
    }

    Vec midpoint() const {
        return (vertices[0] + vertices[1] + vertices[2]) / 3.f;
    }

    // Check if triangle lies in the plane defined by the normal ax through 0.
    bool is_planar(Axis ax) const {
        if (ax == Axis::X) {
            return eps_zero(normal.y) && eps_zero(normal.z);
        } else if (ax == Axis::Y) {
            return eps_zero(normal.x) && eps_zero(normal.z);
        }
        return eps_zero(normal.x) && eps_zero(normal.y);
    }

    // members
    std::array<Vec, 3> vertices;
    std::array<Vec, 3> normals;
    aiColor4D ambient;
    aiColor4D diffuse;

    // precomputed
    // Edges of the triangle from point 0 to points 1 resp. 2
    Vec u, v;
    // Normal vector of the triangle
    // Note: normals of the vertices may be different to this vector, if
    // the triangle is not rendered with sharp edges, i.e. if the normals
    // of the vertices are interpolated between all faces containing this
    // vertex.
    Vec normal;
private:
    float uv, vv, uu, denom;

private:

    // Helper functions for triangle aabb intersection
    bool axis_intersection(const Axis ax, const Vec& box_halfsize) const;
};

using Triangles = std::vector<Triangle>;
