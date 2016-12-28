#pragma once

#include "functional.h"
#include "triangle.h"
#include "types.h"

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <array>
#include <unordered_map>

using RadiosityMesh = OpenMesh::PolyMesh_ArrayKernelT<>;

// Property to store the vertices describing corners of a triangle
// Since after subdivision, a triangle may consists of more than 3 vertices, we
// have to keep track of the corners.
using CornerVertices =
    OpenMesh::FPropHandleT<std::array<RadiosityMesh::VertexHandle, 3>>;
using CornerVerticesProperty =
    OpenMesh::PropertyManager<CornerVertices, RadiosityMesh>;

using VertexRadiosityHandle = OpenMesh::VPropHandleT<Color>;
using VertexRadiosityHandleProperty =
    OpenMesh::PropertyManager<VertexRadiosityHandle, RadiosityMesh>;

using FaceRadiosityHandle = OpenMesh::FPropHandleT<Color>;
using FaceRadiosityHandleProperty =
    OpenMesh::PropertyManager<FaceRadiosityHandle, RadiosityMesh>;

namespace detail {
auto get_halfedge_handle(const RadiosityMesh& mesh,
                         RadiosityMesh::FaceHandle face, size_t index) {
    auto it = mesh.cfh_iter(face);
    while (index--) {
        it++;
    }
    return *it;
}

auto find_vertex(const RadiosityMesh& mesh, RadiosityMesh::FaceHandle face,
                 const RadiosityMesh::Point& pt) {
    size_t index = 0;
    for (auto v : mesh.fv_range(face)) {
        if (mesh.point(v) == pt) {
            break;
        }
        index++;
    }
    return index;
}
} // namespace detail

auto build_mesh(const Triangles& triangles) {
    RadiosityMesh mesh;
    // a vertex is uniquely determined by its coordinates and a normal
    std::unordered_map<std::pair<Vec, Vec>, RadiosityMesh::VertexHandle> lookup;

    auto corners_prop =
        CornerVerticesProperty::createIfNotExists(mesh, "corner_vertices");

    std::array<RadiosityMesh::VertexHandle, 3> vhandles;
    for (const auto& tri : triangles) {
        const Vec& a = tri.vertices[0];
        const Vec& b = tri.vertices[1];
        const Vec& c = tri.vertices[2];
        const Vec& na = tri.normals[0];
        const Vec& nb = tri.normals[1];
        const Vec& nc = tri.normals[2];

        auto ita = lookup.emplace(std::make_pair(a, na),
                                  RadiosityMesh::VertexHandle());
        if (ita.second) {
            ita.first->second = mesh.add_vertex({a.x, a.y, a.z});
        }
        auto va = ita.first->second;

        auto itb = lookup.emplace(std::make_pair(b, nb),
                                  RadiosityMesh::VertexHandle());
        if (itb.second) {
            itb.first->second = mesh.add_vertex({b.x, b.y, b.z});
        }
        auto vb = itb.first->second;

        auto itc = lookup.emplace(std::make_pair(c, nc),
                                  RadiosityMesh::VertexHandle());
        if (itc.second) {
            itc.first->second = mesh.add_vertex({c.x, c.y, c.z});
        }
        auto vc = itc.first->second;

        auto face = mesh.add_face(va, vb, vc);
        corners_prop[face] = {va, vb, vc};
    }

    return mesh;
}

auto subdivide4(RadiosityMesh& mesh, RadiosityMesh::FaceHandle face) {
    CornerVerticesProperty corner_vertices_prop(mesh, "corner_vertices", true);

    // build index of corners and rearrange their coordinates accordingly
    const auto corners = corner_vertices_prop[face];
    std::array<RadiosityMesh::VertexHandle, 3>
        corner_vertices;                // corner vertices
    std::array<size_t, 3> corner_index; // corner indices
    size_t index = 0;
    size_t current = 0;
    for (auto v : mesh.fv_range(face)) {
        if (v == corners[0] || v == corners[1] || v == corners[2]) {
            corner_index[current] = index;
            corner_vertices[current] = v;
            current++;
        }
        index++;
    }
    assert(current == 3);
    size_t N = index;

    size_t i0 = corner_index[0];
    size_t i1 = corner_index[1];
    size_t i2 = corner_index[2];

    // prepare for addition of middle points
    const auto& a = mesh.point(corner_vertices[0]);
    const auto& b = mesh.point(corner_vertices[1]);
    const auto& c = mesh.point(corner_vertices[2]);

    const auto mab = (a + b) / 2;
    const auto mbc = (b + c) / 2;
    const auto mac = (a + c) / 2;

    // add pt between i0 and i1
    size_t k0 = detail::find_vertex(mesh, face, mab);
    if (k0 == N) {
        // no vertex found, i.e. neighbor face was not divided yet
        k0 = i0 + 1;
        auto he = detail::get_halfedge_handle(mesh, face, k0);
        auto edge = mesh.edge_handle(he);
        mesh.split(edge, mab);
        N += 1;
        i1 += 1;
        i2 += 1;
    }

    // add pt between i1 and i2
    size_t k1 = detail::find_vertex(mesh, face, mbc);
    if (k1 == N) {
        // no vertex found, i.e. neighbor face was not divided yet
        k1 = i1 + 1;
        auto he = detail::get_halfedge_handle(mesh, face, k1);
        auto edge = mesh.edge_handle(he);
        mesh.split(edge, mbc);
        N += 1;
        i2 += 1;
    }

    // add pt between i2 and i0
    size_t k2 = detail::find_vertex(mesh, face, mac);
    if (k2 == N) {
        // no vertex found, i.e. neighbor face was not divided yet
        k2 = i2 + 1;
        auto he = detail::get_halfedge_handle(mesh, face, k2 % N);
        auto edge = mesh.edge_handle(he);
        mesh.split(edge, mac);
        N += 1;

        auto first_vertex = *mesh.fv_iter(face);
        if (first_vertex != corner_vertices[0]) {
            i0 += 1;
            i1 += 1;
            i2 += 1;
            k0 += 1;
            k1 += 1;
            k2 = 0;
        }
    }

    // collect halfedges before and after midpoints
    std::array<RadiosityMesh::HalfedgeHandle, 6> halfedges;
    index = 0;
    current = 0;
    for (auto it = mesh.cfh_iter(face); it.is_valid(); ++it) {
        if (index == k0) {
            halfedges[0] = *it;
            halfedges[1] = *(++it);
            current++;
            index++;
        } else if (index == k1) {
            halfedges[2] = *it;
            halfedges[3] = *(++it);
            current++;
            index++;
        } else if (index == k2) {
            halfedges[4] = *it;
            halfedges[5] = *(++it);
            current++;
            index++;
        }
        index++;
    }
    assert(current == 3);

    // subdivide
    std::array<RadiosityMesh::FaceHandle, 4> faces;
    auto he_1 = mesh.insert_edge(halfedges[0], halfedges[5]);
    faces[1] = RadiosityMesh::FaceHandle(mesh.n_faces() - 1);
    corner_vertices_prop[faces[1]] = {corner_vertices[0],
                                      mesh.from_vertex_handle(he_1),
                                      mesh.to_vertex_handle(he_1)};

    auto he_2 = mesh.insert_edge(halfedges[2], halfedges[1]);
    faces[2] = RadiosityMesh::FaceHandle(mesh.n_faces() - 1);
    corner_vertices_prop[faces[2]] = {corner_vertices[1],
                                      mesh.from_vertex_handle(he_2),
                                      mesh.to_vertex_handle(he_2)};

    auto he_3 = mesh.insert_edge(halfedges[4], halfedges[3]);
    faces[3] = RadiosityMesh::FaceHandle(mesh.n_faces() - 1);
    corner_vertices_prop[faces[3]] = {corner_vertices[2],
                                      mesh.from_vertex_handle(he_3),
                                      mesh.to_vertex_handle(he_3)};

    faces[0] = face;
    corner_vertices_prop[faces[0]] = {mesh.to_vertex_handle(he_1),
                                      mesh.to_vertex_handle(he_2),
                                      mesh.to_vertex_handle(he_3)};

    return faces;
}

const auto& triangle_vertices(const RadiosityMesh& mesh,
                              const RadiosityMesh::FaceHandle face) {
    CornerVertices prop_handle;
    mesh.get_property_handle(prop_handle, "corner_vertices");
    const auto& corners = mesh.property(prop_handle, face);
    return corners;
}

auto triangle_midpoint(const RadiosityMesh& mesh,
                       const std::array<RadiosityMesh::VertexHandle, 3>& vs) {
    const auto& a = mesh.point(vs[0]);
    const auto& b = mesh.point(vs[1]);
    const auto& c = mesh.point(vs[2]);
    return (a + b + c) / 3.f;
}

auto triangle_normal(const RadiosityMesh& mesh,
                     const std::array<RadiosityMesh::VertexHandle, 3>& vs) {
    const auto& a = mesh.point(vs[0]);
    const auto& b = mesh.point(vs[1]);
    const auto& c = mesh.point(vs[2]);
    return ((b - a) % (c - a)).normalize();
}
