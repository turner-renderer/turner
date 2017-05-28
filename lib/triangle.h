#pragma once

#include "geometry.h"
#include "turner.h"
#include "types.h"

#include <cereal/types/array.hpp>
#include <cereal/types/vector.hpp>

#include <array>
#include <vector>

using namespace turner;

// TODO: Move in a separate file.
namespace cereal {
template <typename Archive> void serialize(Archive& archive, Vector3f& n) {
    archive(n.x, n.y, n.z);
}

template <typename Archive> void serialize(Archive& archive, Point3f& v) {
    archive(v.x, v.y, v.z);
}

template <typename Archive> void serialize(Archive& archive, Normal3f& n) {
    archive(n.x, n.y, n.z);
}

template <typename Archive> void serialize(Archive& archive, Bbox3f& b) {
    archive(b.p_min, b.p_max);
}
} // namespace cereal

//
// Do NOT modify data in the triangle after its construction! The precomputed
// values won't be updated.
//
// TODO: To refactor!
//
class Triangle {
public:
    Triangle() = default;

    Triangle(std::array<Point3f, 3> vs, std::array<Normal3f, 3> ns,
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
        , normal(normalize(cross(u, v)))
        , uv(dot(u, v))
        , vv(dot(v, v))
        , uu(dot(u, u))
        , denom(uv * uv - uu * vv) {}

    // minimal constructor
    explicit Triangle(std::array<Point3f, 3> vs)
        : Triangle(vs, {Normal3f{}, Normal3f{}, Normal3f{}}, {}, {}, {}, {},
                   0) {}

    friend bool intersect_ray_triangle(const Ray& ray, const Triangle& tri,
                                       float& r, float& s, float& t);

    /**
     * Interpolate normal using barycentric coordinates.
     *
     * Requirement: r + s + t == 1
     */
    Normal3f interpolate_normal(float r, float s, float t) const {
        Normal3f n = r * normals[0] + s * normals[1] + t * normals[2];
        return normalize(n);
    }

    /**
     * Compute the (axes aligned) bounding box of the triangle.
     */
    Bbox3f bbox() const {
        Point3f min = {fmin(vertices[0].x, vertices[1].x, vertices[2].x),
                       fmin(vertices[0].y, vertices[1].y, vertices[2].y),
                       fmin(vertices[0].z, vertices[1].z, vertices[2].z)};

        Point3f max = {fmax(vertices[0].x, vertices[1].x, vertices[2].x),
                       fmax(vertices[0].y, vertices[1].y, vertices[2].y),
                       fmax(vertices[0].z, vertices[1].z, vertices[2].z)};

        return {min, max};
    }

    Point3f midpoint() const {
        Vector3f v0(vertices[0]);
        Vector3f v1(vertices[0]);
        Vector3f v2(vertices[0]);
        return Point3f((v0 + v1 + v2) / 3.f);
    }

    float area() const { return cross(u, v).length() / 2.f; }

    // Check if triangle lies in the plane defined by the normal ax through 0.
    bool is_planar(Axis ax) const {
        if (ax == Axis::X) {
            return is_eps_zero(normal.y) && is_eps_zero(normal.z);
        } else if (ax == Axis::Y) {
            return is_eps_zero(normal.x) && is_eps_zero(normal.z);
        }
        return is_eps_zero(normal.x) && is_eps_zero(normal.y);
    }

    template <class Archive> void serialize(Archive& archive) {
        archive(vertices, normals, ambient, diffuse, emissive, reflective,
                reflectivity, u, v, normal, uv, vv, uu, denom);
    }

    // members
    std::array<Point3f, 3> vertices;
    std::array<Normal3f, 3> normals;
    aiColor4D ambient;
    aiColor4D diffuse;
    aiColor4D emissive;
    aiColor4D reflective;
    float reflectivity;

    // precomputed
    // Edges of the triangle from point 0 to points 1 resp. 2
    Vector3f u, v;
    // Normal vector of the triangle
    // Note: normals of the vertices may be different to this vector, if
    // the triangle is not rendered with sharp edges, i.e. if the normals
    // of the vertices are interpolated between all faces containing this
    // vertex.
    Normal3f normal;

private:
    float uv, vv, uu, denom;

private:
    // Helper functions for triangle aabb intersection
    bool axis_intersection(const Axis ax, const Vector3f& box_halfsize) const;
};

using Triangles = std::vector<Triangle>;
