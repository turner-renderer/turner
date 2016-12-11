#include "../lib/mesh.h"
#include <catch.hpp>

using Points = std::set<RadiosityMesh::Point>;

auto get_points(const RadiosityMesh& mesh, RadiosityMesh::FaceHandle face) {
    Points points;
    for (auto v : mesh.fv_range(face)) {
        points.insert(mesh.point(v));
    }
    return points;
}

auto get_corner_points(RadiosityMesh& mesh, RadiosityMesh::FaceHandle face) {
    auto corner_vertices_prop =
        CornerVerticesProperty(mesh, "corner_vertices", true);
    std::array<RadiosityMesh::VertexHandle, 3> corner_vertices =
        corner_vertices_prop[face];

    Points points;
    for (auto v : corner_vertices) {
        points.insert(mesh.point(v));
    }
    return points;
}

auto num_vertices(const RadiosityMesh& mesh, RadiosityMesh::FaceHandle face) {
    size_t N = 0;
    for (auto v : mesh.fv_range(face)) {
        N++;
    }
    return N;
}

void check_subdivision(RadiosityMesh& mesh,
                       std::array<RadiosityMesh::FaceHandle, 4>& faces,
                       RadiosityMesh::Point a, RadiosityMesh::Point b,
                       RadiosityMesh::Point c, RadiosityMesh::Point ma,
                       RadiosityMesh::Point mb, RadiosityMesh::Point mc) {
    {
        auto pts = get_points(mesh, faces[0]);
        REQUIRE(pts == (Points{ma, mb, mc}));
        auto corner_pts = get_corner_points(mesh, faces[0]);
        REQUIRE(corner_pts == (Points{ma, mb, mc}));
    }
    {
        auto pts = get_points(mesh, faces[1]);
        REQUIRE(pts == (Points{a, mc, mb}));
        auto corner_pts = get_corner_points(mesh, faces[1]);
        REQUIRE(corner_pts == (Points{a, mc, mb}));
    }
    {
        auto pts = get_points(mesh, faces[2]);
        REQUIRE(pts == (Points{b, ma, mc}));
        auto corner_pts = get_corner_points(mesh, faces[2]);
        REQUIRE(corner_pts == (Points{b, ma, mc}));
    }
    {
        auto pts = get_points(mesh, faces[3]);
        REQUIRE(pts == (Points{c, ma, mb}));
        auto corner_pts = get_corner_points(mesh, faces[3]);
        REQUIRE(corner_pts == (Points{c, ma, mb}));
    }
}

TEST_CASE("Simple subdivide4", "[subdivide4]") {
    RadiosityMesh mesh;
    auto corner_vertices =
        CornerVerticesProperty::createIfNotExists(mesh, "corner_vertices");

    RadiosityMesh::Point a(0, 0, 0);
    RadiosityMesh::Point b(1, 0, 0);
    RadiosityMesh::Point c(0, 1, 0);

    auto va = mesh.add_vertex(a);
    auto vb = mesh.add_vertex(b);
    auto vc = mesh.add_vertex(c);
    auto face = mesh.add_face(va, vb, vc);
    corner_vertices[face] = {va, vb, vc};

    auto faces = subdivide4(mesh, face);

    auto ma = (b + c) / 2;
    auto mb = (a + c) / 2;
    auto mc = (a + b) / 2;

    REQUIRE(faces[0] == face);
    check_subdivision(mesh, faces, a, b, c, ma, mb, mc);
}

TEST_CASE("Square subdivide4", "[subdivide4]") {
    RadiosityMesh mesh;
    auto corner_vertices =
        CornerVerticesProperty::createIfNotExists(mesh, "corner_vertices");

    RadiosityMesh::Point a(0, 0, 0);
    RadiosityMesh::Point b(1, 0, 0);
    RadiosityMesh::Point c(0, 1, 0);
    RadiosityMesh::Point d(1, 1, 0);

    auto va = mesh.add_vertex(a);
    auto vb = mesh.add_vertex(b);
    auto vc = mesh.add_vertex(c);
    auto vd = mesh.add_vertex(d);
    auto face_1 = mesh.add_face(va, vb, vc);
    auto face_2 = mesh.add_face(vb, vd, vc);
    corner_vertices[face_1] = {va, vb, vc};
    corner_vertices[face_2] = {vb, vd, vc};

    auto mab = (a + b) / 2;
    auto mbc = (b + c) / 2;
    auto mac = (a + c) / 2;
    auto mbd = (b + d) / 2;
    auto mcd = (c + d) / 2;

    auto faces_2 = subdivide4(mesh, face_2);

    check_subdivision(mesh, faces_2, b, d, c, mcd, mbc, mbd);
    REQUIRE(num_vertices(mesh, face_1) == 4);

    auto faces_1 = subdivide4(mesh, face_1);
    REQUIRE(mesh.n_faces() == 8);
    check_subdivision(mesh, faces_1, a, b, c, mbc, mac, mab);
    check_subdivision(mesh, faces_2, b, d, c, mcd, mbc, mbd);
}

TEST_CASE("Complex square subdivide4", "[subdivide4]") {
    RadiosityMesh mesh;
    auto corner_vertices =
        CornerVerticesProperty::createIfNotExists(mesh, "corner_vertices");

    RadiosityMesh::Point a(0, 0, 0);
    RadiosityMesh::Point b(1, 0, 0);
    RadiosityMesh::Point c(0, 1, 0);
    RadiosityMesh::Point d(1, 1, 0);

    auto va = mesh.add_vertex(a);
    auto vb = mesh.add_vertex(b);
    auto vc = mesh.add_vertex(c);
    auto vd = mesh.add_vertex(d);
    auto face_1 = mesh.add_face(va, vb, vc);
    auto face_2 = mesh.add_face(vb, vd, vc);
    corner_vertices[face_1] = {va, vb, vc};
    corner_vertices[face_2] = {vb, vd, vc};

    auto mab = (a + b) / 2;
    auto mbc = (b + c) / 2;
    auto mac = (a + c) / 2;
    auto mbd = (b + d) / 2;
    auto mcd = (c + d) / 2;

    auto faces = subdivide4(mesh, face_2);
    for (auto f : faces) {
        subdivide4(mesh, f);
    }

    REQUIRE(num_vertices(mesh, face_1) == 6);
    REQUIRE(mesh.n_faces() == 1 + 4 * 4);

    faces = subdivide4(mesh, face_1);
    REQUIRE(mesh.n_faces() == 4 + 4 * 4);

    {
        REQUIRE(num_vertices(mesh, faces[0]) == 3);
        auto pts = get_corner_points(mesh, faces[0]);
        REQUIRE(pts == (Points{mab, mbc, mac}));
    }
    {
        REQUIRE(num_vertices(mesh, faces[1]) == 3);
        auto pts = get_corner_points(mesh, faces[1]);
        REQUIRE(pts == (Points{a, mab, mac}));
    }
    {
        REQUIRE(num_vertices(mesh, faces[2]) == 4);
        auto pts = get_points(mesh, faces[2]);
        REQUIRE(pts == (Points{mab, b, b + (c - b) / 4, mbc}));
        auto corner_pts = get_corner_points(mesh, faces[2]);
        REQUIRE(corner_pts == (Points{mab, b, mbc}));
    }
    {
        REQUIRE(num_vertices(mesh, faces[3]) == 4);
        auto pts = get_points(mesh, faces[3]);
        REQUIRE(pts == (Points{mac, c, c + (b - c) / 4, mbc}));
        auto corner_pts = get_corner_points(mesh, faces[3]);
        REQUIRE(corner_pts == (Points{mac, c, mbc}));
    }
}
