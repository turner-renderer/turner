#include "../lib/types.h"
#include "../lib/intersection.h"
#include "catch.hpp"
#include <iostream>


TEST_CASE("Segment plane intersection", "[intersection]") {
    float t;
    REQUIRE(
        intersect_segment_plane({0, 0, 0}, {2, 0, 0}, {1, 0, 0}, 1, t));
    REQUIRE(t == 0.5f);
}


TEST_CASE("Ray AABB intersection", "[intersection]") {
    REQUIRE(ray_box_intersection(
        Ray({0, 0, 0}, {1, 1, 1}),
        Box{{-1, -1, -1}, {1, 1, 1}}));

    REQUIRE(ray_box_intersection(
        Ray({10, 0, 0}, {-1, 0, 0}),
        Box{{-1, -1, -1}, {1, 1, 1}}));

    REQUIRE(ray_box_intersection(
        Ray({0, 10, 0}, {0, -1, 0}),
        Box{{-1, -1, -1}, {1, 1, 1}}));

    REQUIRE(ray_box_intersection(
        Ray({0, 0, 10}, {0, 0, -1}),
        Box{{-1, -1, -1}, {1, 1, 1}}));

    REQUIRE(!ray_box_intersection(
        Ray({0, 0, 0}, {1, 0, 0}),
        Box{{-1, -1, 1}, {1, 1, 1}}));

    REQUIRE(!ray_box_intersection(
        Ray({-2, -2, -2}, {-1, 0, 0}),
        Box{{-1, -1, 1}, {1, 1, 1}}));

    REQUIRE(!ray_box_intersection(
        Ray({-1, 0, 0}, {-1, 0, 0}),
        Box{{0, 0, 0}, {1, 1, 1}}));
}
