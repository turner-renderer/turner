#include "../lib/intersection.h"
#include "helper.h"
#include "catch.hpp"
#include <iostream>


TEST_CASE("Segment plane intersection", "[intersection]") {
    float t;
    REQUIRE(
        intersect_segment_plane({0, 0, 0}, {2, 0, 0}, {1, 0, 0}, 1, t));
    REQUIRE(t == 0.5f);
}


TEST_CASE("Ray AABB intersection", "[intersection]") {
    REQUIRE(intersect_ray_box(Ray({0, 0, 0}, {1, 1, 1}),
                              Bbox3f{{-1, -1, -1}, {1, 1, 1}}));

    REQUIRE(intersect_ray_box(Ray({10, 0, 0}, {-1, 0, 0}),
                              Bbox3f{{-1, -1, -1}, {1, 1, 1}}));

    REQUIRE(intersect_ray_box(Ray({0, 10, 0}, {0, -1, 0}),
                              Bbox3f{{-1, -1, -1}, {1, 1, 1}}));

    REQUIRE(intersect_ray_box(Ray({0, 0, 10}, {0, 0, -1}),
                              Bbox3f{{-1, -1, -1}, {1, 1, 1}}));

    REQUIRE(!intersect_ray_box(Ray({0, 0, 0}, {1, 0, 0}),
                               Bbox3f{{-1, -1, 1}, {1, 1, 1}}));

    REQUIRE(!intersect_ray_box(Ray({-2, -2, -2}, {-1, 0, 0}),
                               Bbox3f{{-1, -1, 1}, {1, 1, 1}}));

    REQUIRE(!intersect_ray_box(Ray({-1, 0, 0}, {-1, 0, 0}),
                               Bbox3f{{0, 0, 0}, {1, 1, 1}}));
}


TEST_CASE("Test intersect plane with AABB", "[intersection]") {
    REQUIRE(intersect_plane_box(
        {1, 0, 0}, 1, {{-10, -10, -10}, {10, 10, 10}}));
    REQUIRE(!intersect_plane_box(
        {1, 0, 0}, 20, {{-10, -10, -10}, {10, 10, 10}}));
    REQUIRE(intersect_plane_box(
        {1, 0, -1}, 0, {{9, 9, 9}, {10, 10, 10}}));
}


TEST_CASE("Triangle simple aabb intersection", "[intersection]") {
    Bbox3f box{{-10, -10, -10}, {10, 10, 10}};

    auto tri = test_triangle({0, 0, 0}, {1, 0, 0}, {1, 1, 0});
    REQUIRE(intersect_triangle_box(tri, box));

    tri = test_triangle({-20, -20, 0}, {-15, -20, 0}, {-15, -15, 0});
    REQUIRE(!intersect_triangle_box(tri, box));

    tri = test_triangle({-10, -10, 10}, {10, -10, 10}, {10, 10, 10});
    REQUIRE(intersect_triangle_box(tri, box));
}


TEST_CASE("Random triangle aabb intersection", "[intersection]") {
    Bbox3f box{{-10, -10, -10}, {10, 10, 10}};
    for (int i = 0; i < 1000; ++i) {
        REQUIRE(intersect_triangle_box(random_triangle(), box));
    }
}
