#pragma once

#include "types.h"
#include "intersection.h"
#include <vector>
#include <array>


struct Triangle {
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

    // members
    const std::array<Vec, 3> vertices;
    const std::array<Vec, 3> normals;
    const aiColor4D ambient;
    const aiColor4D diffuse;

    // precomputed
    // Edges of the triangle from point 0 to points 1 resp. 2
    const Vec u, v;
    // Normal vector of the triangle
    // Note: normals of the vertices may be different to this vector, if
    // the triangle is not rendered with sharp edges, i.e. if the normals
    // of the vertices are interpolated between all faces containing this
    // vertex.
    const Vec normal;
private:
    float uv, vv, uu, denom;
};

using Triangles = std::vector<Triangle>;
