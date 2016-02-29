#include "helper.h"
#include "../lib/clipping.h"
#include "../lib/output.h"
#include <catch.hpp>


TEST_CASE("Simple clipping test", "[clipping]") {
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


TEST_CASE("Simple triangle clipping at aabb", "[clipping]") {
    Box box{{-1, -1, -1}, {1, 1, 1}};

    auto tri = test_triangle({-1, -1, -1}, {2, 2, 2}, {2, -1, 2});
    auto res = triangle_clip_aabb(tri, box);
    REQUIRE(res == box);

    tri = test_triangle({0, 0, 0}, {0.5, 0.5, 0.5}, {0.5, 0, 0.5});
    res = triangle_clip_aabb(tri, box);
    REQUIRE(res == (Box{{0, 0, 0}, {0.5, 0.5, 0.5}}));
}

TEST_CASE("Random triangle clipping at aabb", "[clipping]") {
    Box box{{-1, -1, -1}, {1, 1, 1}};

    for (int i = 0; i < 1000; ++i) {
        // triangle in box [(-10, -10, -10), (10, 10, 10)]
        auto tri = random_triangle();
        auto res = triangle_clip_aabb(tri, box);

        REQUIRE(box.min.x <= res.min.x);
        REQUIRE(res.max.x <= box.max.x);
        REQUIRE(box.min.y <= res.min.y);
        REQUIRE(res.max.y <= box.max.y);
        REQUIRE(box.min.z <= res.min.z);
        REQUIRE(res.max.z <= box.max.z);
    }
}
