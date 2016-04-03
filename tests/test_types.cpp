#include "helper.h"
#include "../lib/types.h"
#include "../lib/triangle.h"
#include "../lib/output.h"
#include "../lib/runtime.h"
#include <catch.hpp>


TEST_CASE("Test min and max", "[helper]") {
    REQUIRE(min(1, 2, 3) == 1);
    REQUIRE(min(2, 1, 3) == 1);
    REQUIRE(min(3, 2, 1) == 1);

    REQUIRE(max(1, 2, 3) == 3);
    REQUIRE(max(2, 1, 3) == 3);
    REQUIRE(max(3, 2, 1) == 3);

    REQUIRE(min(1, 1, 1) == 1);
    REQUIRE(max(1, 1, 1) == 1);
}


TEST_CASE("Test surface area of box", "[box]") {
    Box box{{0, 0, 0}, {1, 1, 1}};
    REQUIRE(box.surface_area() == 6);

    box = Box{{-10, -10, -10}, {10, 10, 10}};
    REQUIRE(box.surface_area() == 2400);

    box = Box{{0, 0, 0}, {1, 1, 0}};
    REQUIRE(box.surface_area() == 4);
}

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

TEST_CASE("Split box at plane", "[box]") {
    Box box{{0, 0, 0}, {2, 2, 2}};

    Box l, r;
    std::tie(l, r) = box.split(Axis::X, 1);
    REQUIRE(l == (Box{{0, 0, 0}, {1, 2, 2}}));
    REQUIRE(r == (Box{{1, 0, 0}, {2, 2, 2}}));

    std::tie(l, r) = box.split(Axis::Y, 1);
    REQUIRE(l == (Box{{0, 0, 0}, {2, 1, 2}}));
    REQUIRE(r == (Box{{0, 1, 0}, {2, 2, 2}}));

    std::tie(l, r) = box.split(Axis::Z, 1);
    REQUIRE(l == (Box{{0, 0, 0}, {2, 2, 1}}));
    REQUIRE(r == (Box{{0, 0, 1}, {2, 2, 2}}));
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
    for (auto ax : AXES) {
        Vec vs[3] = {random_vec(), random_vec(), random_vec()};
        for (int i = 0; i < 3; ++i) {
            vs[i][ax] = 0.f;
        }

        Triangle triangle = test_triangle(vs[0], vs[1], vs[2]);
        REQUIRE(triangle.is_planar(ax));
    }
}
