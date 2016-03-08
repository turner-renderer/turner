#include "helper.h"
#include "../lib/clipping.h"
#include "../lib/output.h"
#include <catch.hpp>


TEST_CASE("Clip polygon at a plane", "[clipping]") {
    auto tri = std::vector<Vec>{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}};
    auto res = clip_polygon_at_plane(tri, {1, 0, 0}, 0);
    REQUIRE(res == tri);

    tri = std::vector<Vec>{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}};
    res = clip_polygon_at_plane(tri, {1, 0, 0}, 0.5f);
    auto expected = std::vector<Vec>{
        {0.5f, 0.5f, 0}, {0.5f, 0, 0}, {1, 0, 0}, {1, 1, 0}};
    REQUIRE(res == expected);

    tri = std::vector<Vec>{{0, 1, 0}, {0, 0, 0}, {1, 0, 0}};
    auto normal = Vec{-1, 1, 0}.Normalize();
    res = clip_polygon_at_plane(tri, normal, 0);
    expected = std::vector<Vec>{{0.5f, 0.5f, 0}, {0, 1, 0}, {0, 0, 0}};
    REQUIRE(res == expected);
}


TEST_CASE("Simple line clipping test", "[clipping]") {
    Box box{{-1, -1, -1}, {1, 1, 1}};

    Vec p0 = {0, 0, -10};
    Vec p1 = {0, 0, 10};

    REQUIRE(clip_line_aabb(p0, p1, box));
    REQUIRE(p0 == (Vec{0, 0, -1}));
    REQUIRE(p1 == (Vec{0, 0, 1}));

    p0 = {0, -10, 0};
    p1 = {0, 10, 0};

    REQUIRE(clip_line_aabb(p0, p1, box));
    REQUIRE(p0 == (Vec{0, -1, 0}));
    REQUIRE(p1 == (Vec{0, 1, 0}));

    p0 = {-10, 0, 0};
    p1 = {10, 0, 0};

    REQUIRE(clip_line_aabb(p0, p1, box));
    REQUIRE(p0 == (Vec{-1, 0, 0}));
    REQUIRE(p1 == (Vec{1, 0, 0}));

    p0 = {-10, 10, 0};
    p1 = {10, 10, 0};

    REQUIRE(!clip_line_aabb(p0, p1, box));
    REQUIRE(p0 == (Vec{-10, 10, 0}));
    REQUIRE(p1 == (Vec{10, 10, 0}));
}


TEST_CASE("Big triangle clipping at aabb", "[clipping]") {
    Box box{{-1, -1, -1}, {1, 1, 1}};
    auto tri = test_triangle({0, 0, -10}, {10, 0, 10}, {-10, 0, 10});
    auto res = clip_triangle_at_aabb(tri, box);
    REQUIRE(res == (Box{{-1, 0, -1}, {1, 0, 1}}));
    REQUIRE(res.is_planar(Axis::Y));
}

TEST_CASE("Peak triangle clipping at aabb", "[clipping]") {
    Box box{{0, 0, 0}, {2, 2, 2}};
    auto tri = test_triangle({-1, -1, 0}, {1, 1, 0}, {1, -1, 0});
    auto res = clip_triangle_at_aabb(tri, box);
    REQUIRE(res == (Box{{0, 0, 0}, {2, 1, 0}}));
    REQUIRE(res.is_planar(Axis::Z));
}


TEST_CASE("Simple triangle clipping at aabb", "[clipping]") {
    Box box{{-1, -1, -1}, {1, 1, 1}};

    auto tri = test_triangle({-1, -1, -1}, {2, 2, 2}, {2, -1, 2});
    auto res = clip_triangle_at_aabb(tri, box);
    REQUIRE(res == box);

    tri = test_triangle({0, 0, 0}, {0.5, 0.5, 0.5}, {0.5, 0, 0.5});
    res = clip_triangle_at_aabb(tri, box);
    REQUIRE(res == (Box{{0, 0, 0}, {0.5, 0.5, 0.5}}));
}

TEST_CASE("Random triangle clipping at aabb", "[clipping]") {
    Box box{{-1, -1, -1}, {1, 1, 1}};
    for (int i = 0; i < 10000; ++i) {
        auto tri = random_triangle();
        if (!tri.intersect(box)) {
            continue;
        }

        auto res = clip_triangle_at_aabb(tri, box);

        REQUIRE(-EPS < res.min.x - box.min.x);
        REQUIRE(-EPS < res.min.y - box.min.y);
        REQUIRE(-EPS < res.min.z - box.min.z);

        REQUIRE(-EPS < box.max.x - res.max.x);
        REQUIRE(-EPS < box.max.y - res.max.y);
        REQUIRE(-EPS < box.max.z - res.max.z);
    }
}
