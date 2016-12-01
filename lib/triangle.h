#pragma once

#include "types.h"

#include <array>
#include <vector>

//
// Do NOT modify data in the triangle after its construction! The precomputed
// values won't be updated.
//
// TODO: To refactor!
//
class Triangle {
public:
    Triangle(std::array<Vec, 3> vs, std::array<Vec, 3> ns,
             const aiColor4D& ambient, const aiColor4D& diffuse,
             const aiColor4D& emissive, const aiColor4D& reflective,
             const float reflectivity)
        : vertices(vs)
        , normals(ns)
        , ambient(ambient)
        , diffuse(diffuse)
        , emissive(emissive)
        , reflective(reflective)     // reflective color
        , reflectivity(reflectivity) // reflectivity factor
        , u(vertices[1] - vertices[0])
        , v(vertices[2] - vertices[0])
        , normal((u ^ v).Normalize())
        , uv(u * v)
        , vv(v * v)
        , uu(u * u)
        , denom(uv * uv - uu * vv) {}

    // minimal constructor
    explicit Triangle(std::array<Vec, 3> vs)
        : Triangle(vs, {Vec{}, Vec{}, Vec{}}, {}, {}, {}, {}, 0) {}

    friend bool intersect_ray_triangle(const Ray& ray, const Triangle& tri,
                                       float& r, float& s, float& t);

    /**
     * Interpolate normal using barycentric coordinates.
     *
     * Requirement: r + s + t == 1
     */
    Vec interpolate_normal(float r, float s, float t) const {
        auto normal = r * normals[0] + s * normals[1] + t * normals[2];
        normal.Normalize();
        return normal;
    }

    /**
     * Compute the (axes aligned) bounding box of the triangle.
     */
    Box bbox() const {
        Vec min = {fmin(vertices[0].x, vertices[1].x, vertices[2].x),
                   fmin(vertices[0].y, vertices[1].y, vertices[2].y),
                   fmin(vertices[0].z, vertices[1].z, vertices[2].z)};

        Vec max = {fmax(vertices[0].x, vertices[1].x, vertices[2].x),
                   fmax(vertices[0].y, vertices[1].y, vertices[2].y),
                   fmax(vertices[0].z, vertices[1].z, vertices[2].z)};

        return {min, max};
    }

    Vec midpoint() const {
        return (vertices[0] + vertices[1] + vertices[2]) / 3.f;
    }

    float area() const { return (u ^ v).Length() / 2.f; }

    // Check if triangle lies in the plane defined by the normal ax through 0.
    bool is_planar(Axis ax) const {
        if (ax == Axis::X) {
            return is_eps_zero(normal.y) && is_eps_zero(normal.z);
        } else if (ax == Axis::Y) {
            return is_eps_zero(normal.x) && is_eps_zero(normal.z);
        }
        return is_eps_zero(normal.x) && is_eps_zero(normal.y);
    }

    // members
    std::array<Vec, 3> vertices;
    std::array<Vec, 3> normals;
    aiColor4D ambient;
    aiColor4D diffuse;
    aiColor4D emissive;
    aiColor4D reflective;
    float reflectivity;

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
