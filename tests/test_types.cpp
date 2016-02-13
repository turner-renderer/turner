#include "helper.h"
#include "../lib/types.h"
#include "../lib/triangle.h"
#include "../lib/kdtree.h"
#include "../lib/output.h"
#include "../lib/runtime.h"
#include <catch.hpp>


TEST_CASE("Test Box union", "[box]")
{
    Box box{{0, 0, 0}, {1, 1, 1}};
    REQUIRE((box + Box{{0, 0, 0}, {0, 0, 0}}) == box);
    REQUIRE((box + Box{{0, 0, 0}, {1, 1, 1}}) == box);
    REQUIRE((box + Box{{0, 0, 0}, {0.5f, 0.5f, 0.5f}}) == box);
    REQUIRE((box + Box{{0, 0, 0}, {2, 1, 1}}) == (Box{{0, 0, 0}, {2, 1, 1}}));
    REQUIRE((box + Box{{0, 0, 0}, {1, 2, 1}}) == (Box{{0, 0, 0}, {1, 2, 1}}));
    REQUIRE((box + Box{{0, 0, 0}, {1, 1, 2}}) == (Box{{0, 0, 0}, {1, 1, 2}}));
    REQUIRE((box + Box{{-1, 0, 0},{1, 1, 1}}) == (Box{{-1, 0, 0},{0, 0, 0}}));
    REQUIRE((box + Box{{0, -1, 0},{1, 1, 1}}) == (Box{{0, -1, 0},{0, 0, 0}}));
    REQUIRE((box + Box{{0, 0, -1},{1, 1, 1}}) == (Box{{0, 0, -1},{0, 0, 0}}));
}


TEST_CASE("Test bbox of triangle", "[triangle]")
{
    REQUIRE(
        test_triangle({0, 0, 0}, {0, 0, 1}, {0, 1, 1}).bbox()
        == (Box{{0, 0, 0}, {0, 1, 1}}));
    REQUIRE(
        test_triangle({-1, 0, 0}, {0, 0, 1}, {0, 1, 1}).bbox()
        == (Box{{-1, 0, 0}, {0, 1, 1}}));
    REQUIRE(
        test_triangle({0, 0, 0}, {0, -1, 1}, {0, 1, 1}).bbox()
        == (Box{{0, -1, 0}, {0, 1, 1}}));
    REQUIRE(
        test_triangle({0, 0, 0}, {0, 0, -1}, {0, 1, 1}).bbox()
        == (Box{{0, 0, -1}, {0, 1, 1}}));
    REQUIRE(
        test_triangle({0, 0, 0}, {0, 0, 1}, {2, 1, 1}).bbox()
        == (Box{{0, 0, 0}, {2, 1, 1}}));
}


TEST_CASE("Test is_planar of triangle", "[triangle]")
{
    Axis ax = Axis::X;
    for (int n = 0; n < 3; ++n) {
        ++ax;

        Vec vs[3] = {random_vec(), random_vec(), random_vec()};
        for (int i = 0; i < 3; ++i) {
            vs[i][static_cast<int>(ax)] = 0.f;
        }

        Triangle triangle = test_triangle(vs[0], vs[1], vs[2]);
        REQUIRE(triangle.is_planar(ax));
    }
}
