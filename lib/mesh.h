#pragma once

#include "functional.h"
#include "triangle.h"
#include "types.h"

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

#include <array>
#include <stack>
#include <unordered_map>

/**
 * Mesh type for storing hierarchical radiosity triangle subdivision.
 *
 * We are not using triangle mesh, since after subdivisition of a triangle, the
 * neighbor triangles are triangles with additional vertices at sides.
 */
using RadiosityMesh = OpenMesh::PolyMesh_ArrayKernelT<>;

/**
 * Mesh property for storing the vertices defining the corners of a triangle.
 *
 * Since after the subdivision, a triangle may consists of more than 3 vertices,
 * we have to keep track of the corners.
 */
using CornerVertices =
    OpenMesh::FPropHandleT<std::array<RadiosityMesh::VertexHandle, 3>>;
using CornerVerticesProperty =
    OpenMesh::PropertyManager<CornerVertices, RadiosityMesh>;

/**
 * Property for storing radiosity values at triangles.
 */
using FaceRadiosityHandle = OpenMesh::FPropHandleT<Color>;
using FaceRadiosityHandleProperty =
    OpenMesh::PropertyManager<FaceRadiosityHandle, RadiosityMesh>;

/**
 * Property for storing radiosity values at triangle corners.
 */
using VertexRadiosityHandle = OpenMesh::VPropHandleT<Color>;
using VertexRadiosityHandleProperty =
    OpenMesh::PropertyManager<VertexRadiosityHandle, RadiosityMesh>;

namespace detail {
/**
 * Get halfedge of a face by index.
 */
auto get_halfedge_handle(const RadiosityMesh& mesh,
                         RadiosityMesh::FaceHandle face, size_t index) {
    auto it = mesh.cfh_iter(face);
    while (index--) {
        it++;
    }
    return *it;
}

/**
 * Get edge of a face by index.
 */
auto get_edge_handle(const RadiosityMesh& mesh, RadiosityMesh::FaceHandle face,
                     size_t index) {
    return mesh.edge_handle(get_halfedge_handle(mesh, face, index));
}

/**
 * Get vertex of a face by its coordinates.
 */
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

// TODO: test
/**
 * Build RadiosityMesh from a set of triangles.
 * @param  triangles The set of triangles, to build the mesh from.
 * @return           RadiosityMesh, with faces corresponging to given
 *                   triangles. A face id corresponds exactly to the position
 *                   of the corresponding triangle.
 */
auto build_mesh(const Triangles& triangles) {
    RadiosityMesh mesh;
    // a vertex is uniquely determined by its coordinates and a normal
    std::unordered_map<std::pair<Point3f, Normal3f>,
                       RadiosityMesh::VertexHandle>
        lookup;

    auto corners_prop =
        CornerVerticesProperty::createIfNotExists(mesh, "corner_vertices");

    std::array<RadiosityMesh::VertexHandle, 3> vhandles;
    for (const auto& tri : triangles) {
        const Point3f& a = tri.vertices[0];
        const Point3f& b = tri.vertices[1];
        const Point3f& c = tri.vertices[2];
        const Normal3f& na = tri.normals[0];
        const Normal3f& nb = tri.normals[1];
        const Normal3f& nc = tri.normals[2];

        auto ita = lookup.emplace(std::make_pair(a, na),
                                  RadiosityMesh::VertexHandle());
        RadiosityMesh::VertexHandle& va = ita.first->second;
        if (ita.second) {
            va = mesh.add_vertex({a.x, a.y, a.z});
        }

        auto itb = lookup.emplace(std::make_pair(b, nb),
                                  RadiosityMesh::VertexHandle());
        RadiosityMesh::VertexHandle& vb = itb.first->second;
        if (itb.second) {
            vb = mesh.add_vertex({b.x, b.y, b.z});
        }

        auto itc = lookup.emplace(std::make_pair(c, nc),
                                  RadiosityMesh::VertexHandle());
        RadiosityMesh::VertexHandle& vc = itc.first->second;
        if (itc.second) {
            vc = mesh.add_vertex({c.x, c.y, c.z});
        }

        auto face = mesh.add_face(va, vb, vc);
        corners_prop[face] = {va, vb, vc};
    }

    return mesh;
}

/**
 * Subdivide the triangle given by `face` into 4 subtriangles.
 *
 * Triangle abc is subdivided in the following way.
 *                        c
 *                        /\
 *                       /  \
 *                      /    \
 *                     /  2   \
 *                    /        \
 *                mb \----------/ ma
 *                  / \        / \
 *                 /   \ face /   \
 *                /     \    /     \
 *               /   0   \  /   1   \
 *              /         \/         \
 *             -----------/\-----------
 *            a           mc          b
 *
 * The face given by `face` is then the middle triangle. The new triangles are
 * added to the mesh and have consecutive ids starting after the last ids in
 * the mesh.
 *
 * TODO: Consider to remove the return value.
 *
 * @param  mesh Mesh storing triangles
 * @param  face Triangle to subdivide
 * @return      list of ids of subdivided triangles, i.e. [face, 0, 1, 2].
 */
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
        auto edge = detail::get_edge_handle(mesh, face, k0);
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
        auto edge = detail::get_edge_handle(mesh, face, k1);
        mesh.split(edge, mbc);
        N += 1;
        i2 += 1;
    }

    // add pt between i2 and i0
    size_t k2 = detail::find_vertex(mesh, face, mac);
    if (k2 == N) {
        // no vertex found, i.e. neighbor face was not divided yet
        k2 = i2 + 1;
        auto edge = detail::get_edge_handle(mesh, face, k2 % N);
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

    // collect halfedges before and after midpoints of edges
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

/**
 * Helper to get the triangle vertices property.
 */
const auto& triangle_vertices(const RadiosityMesh& mesh,
                              const RadiosityMesh::FaceHandle face) {
    CornerVertices prop_handle;
    mesh.get_property_handle(prop_handle, "corner_vertices");
    const auto& corners = mesh.property(prop_handle, face);
    return corners;
}

/**
 * Compute triangle midpoint of the triangle given by vertices `vs`.
 */
auto triangle_midpoint(const RadiosityMesh& mesh,
                       const std::array<RadiosityMesh::VertexHandle, 3>& vs) {
    const auto& a = mesh.point(vs[0]);
    const auto& b = mesh.point(vs[1]);
    const auto& c = mesh.point(vs[2]);
    return (a + b + c) / 3.f;
}

/**
 * Compute triangle normal given by vertices `vs`.
 */
auto triangle_normal(const RadiosityMesh& mesh,
                     const std::array<RadiosityMesh::VertexHandle, 3>& vs) {
    const auto& a = mesh.point(vs[0]);
    const auto& b = mesh.point(vs[1]);
    const auto& c = mesh.point(vs[2]);
    return ((b - a) % (c - a)).normalize();
}

// TODO: Test
/**
 * Find a T-vertex on a triangle and triangulate it.
 *
 * The triangle is split at a T-vertex, which has a corner vertex as a neighbor.
 * A new triangle is added, defined by three points: the T-vertex and the two
 * corners of the original triangle. The original triangle and the new triangle
 * may still contain another T-vertices.
 *
 * @param  mesh Mesh containing the triangles
 * @param  face Triangle
 * @return      true, if the triangle is fully triangulated, i.e. does not
 *              contain any T-vertices anymore; otherwise false.
 */
bool triangulate_t_vertex(RadiosityMesh& mesh, RadiosityMesh::FaceHandle face) {
    CornerVerticesProperty corner_vertices_prop(mesh, "corner_vertices", true);

    // build corners index and compute number of vertices
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

    if (N == 3) {
        return true; // is already fully triangulated
    }

    // find a corner s.t. there is a vertex between it and the next corner, and
    // rotate it to the first position
    if (corner_index[1] - corner_index[0] > 1) {
        // nothing to do
    } else if (corner_index[2] - corner_index[1] > 1) {
        std::rotate(corner_index.begin(), corner_index.begin() + 1,
                    corner_index.end());
        std::rotate(corner_vertices.begin(), corner_vertices.begin() + 1,
                    corner_vertices.end());
    } else if (N - corner_index[2] + corner_index[0] > 1) {
        std::rotate(corner_index.begin(), corner_index.begin() + 2,
                    corner_index.end());
        std::rotate(corner_vertices.begin(), corner_vertices.begin() + 2,
                    corner_vertices.end());
    } else {
        assert(!"logic error");
    }

    // find halfedges after the first corner, and at the third corner
    index = 0;
    RadiosityMesh::HalfedgeHandle he_from, he_to;
    for (auto he : mesh.fh_range(face)) {
        if (index == (corner_index[0] + 1) % N) {
            he_from = he;
        } else if (index == (corner_index[2] + 1) % N) {
            he_to = he;
        }
        index++;
    }
    assert(mesh.from_vertex_handle(he_from) == corner_vertices[0]);
    assert(mesh.from_vertex_handle(he_to) == corner_vertices[2]);

    // subdivide this face by connecting the halfedges
    mesh.insert_edge(he_from, he_to);

    // update corners
    auto new_face = RadiosityMesh::FaceHandle(mesh.n_faces() - 1);
    corner_vertices_prop[new_face] = {
        corner_vertices[0], mesh.to_vertex_handle(he_from), corner_vertices[2]};
    corner_vertices_prop[face] = {mesh.to_vertex_handle(he_from),
                                  corner_vertices[1], corner_vertices[2]};

    return N == 4; // is the face fully triangulated?
}

/**
 * Triangulate all T-vertices of the triangle given by `face`.
 */
void triangulate_t_vertices(RadiosityMesh& mesh,
                            RadiosityMesh::FaceHandle face) {
    std::stack<RadiosityMesh::FaceHandle> to_triangulate;
    to_triangulate.push(face);
    while (!to_triangulate.empty()) {
        auto f = to_triangulate.top();
        to_triangulate.pop();
        while (!triangulate_t_vertex(mesh, f)) {
            to_triangulate.push(RadiosityMesh::FaceHandle(mesh.n_faces() - 1));
        }
    }
}
