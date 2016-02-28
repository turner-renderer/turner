#include "helper.h"
#include "../lib//output.h"
#include "../lib/kdtree.h"
#include <catch.hpp>

#include <iostream>


TEST_CASE("Split box at plane", "[box]") {
    Box box{{0, 0, 0}, {2, 2, 2}};

    Box l, r;
    std::tie(l, r) = split(box, Plane{Axis::X, 1});
    REQUIRE(l == (Box{{0, 0, 0}, {1, 2, 2}}));
    REQUIRE(r == (Box{{1, 0, 0}, {2, 2, 2}}));

    std::tie(l, r) = split(box, Plane{Axis::Y, 1});
    REQUIRE(l == (Box{{0, 0, 0}, {2, 1, 2}}));
    REQUIRE(r == (Box{{0, 1, 0}, {2, 2, 2}}));

    std::tie(l, r) = split(box, Plane{Axis::Z, 1});
    REQUIRE(l == (Box{{0, 0, 0}, {2, 2, 1}}));
    REQUIRE(r == (Box{{0, 0, 1}, {2, 2, 2}}));
}


TEST_CASE("Smoke test", "[kdtree]")
{
    auto tri = random_triangle();
    FastKDTree kdtree({tri});
}
