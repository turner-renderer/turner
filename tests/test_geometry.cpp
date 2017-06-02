#include "../src/geometry.h"
#include <catch.hpp>
#include <iostream>

using namespace turner;

TEST_CASE("Vector3 default contructor", "[vector3]") {
    Vector3f v;
    REQUIRE(v.x == 0);
    REQUIRE(v.y == 0);
    REQUIRE(v.z == 0);
}

TEST_CASE("Vector3 contructor", "[vector3]") {
    Vector3f v(-1, 0, 1);
    REQUIRE(v.x == -1);
    REQUIRE(v.y == 0);
    REQUIRE(v.z == 1);
}

TEST_CASE("Vector3 is abelian group", "[vector3]") {
    Vector3f zero;
    Vector3f v(-1, 0, 1);
    Vector3f w(-1, 1, 1);
    Vector3f u(-1, -1, 1);
    REQUIRE(v + zero == v);
    REQUIRE(v - v == zero);
    REQUIRE(v + w == v + w);
    REQUIRE(((v + w) + u) == (v + (w + u)));
}

TEST_CASE("Iterate through integer-valued bbox", "[bbox]") {
    Bbox2i b({-1, -2}, {3, 4});
    int x = 0;
    int y = 0;
    for (const auto& p : b) {
        x += p.x;
        y += p.y;
    }

    REQUIRE(x == (-1 + 0 + 1 + 2) * 6);
    REQUIRE(y == (-2 + -1 + 0 + 1 + 2 + 3) * 4);
}

TEST_CASE("Test surface area of box", "[bbox3]") {
    Bbox3i box{{0, 0, 0}, {1, 1, 1}};
    REQUIRE(box.surface_area() == 6);

    box = Bbox3i{{-10, -10, -10}, {10, 10, 10}};
    REQUIRE(box.surface_area() == 2400);

    box = Bbox3i{{-1, -1, 0}, {1, 1, 0}};
    REQUIRE(box.planar(2));
    REQUIRE(box.surface_area() == 8);

    box = Bbox3i{{0, -1, -1}, {0, 1, 1}};
    REQUIRE(box.surface_area() == 8);

    box = Bbox3i{{-1, 0, -1}, {1, 0, 1}};
    REQUIRE(box.surface_area() == 8);
}

TEST_CASE("Test Bbox3f union", "[bbox3]") {
    Bbox3f box{{0, 0, 0}, {1, 1, 1}};
    REQUIRE(bbox_union(box, Bbox3f{{0, 0, 0}, {0, 0, 0}}) == box);
    REQUIRE(bbox_union(box, Bbox3f{{0, 0, 0}, {1, 1, 1}}) == box);
    REQUIRE(bbox_union(box, Bbox3f{{0, 0, 0}, {0.5f, 0.5f, 0.5f}}) == box);
    REQUIRE(bbox_union(box, Bbox3f{{0, 0, 0}, {2, 1, 1}}) ==
            (Bbox3f{{0, 0, 0}, {2, 1, 1}}));
    REQUIRE(bbox_union(box, Bbox3f{{0, 0, 0}, {1, 2, 1}}) ==
            (Bbox3f{{0, 0, 0}, {1, 2, 1}}));
    REQUIRE(bbox_union(box, Bbox3f{{0, 0, 0}, {1, 1, 2}}) ==
            (Bbox3f{{0, 0, 0}, {1, 1, 2}}));
    REQUIRE(bbox_union(box, Bbox3f{{-1, 0, 0}, {1, 1, 1}}) ==
            (Bbox3f{{-1, 0, 0}, {1, 1, 1}}));
    REQUIRE(bbox_union(box, Bbox3f{{0, -1, 0}, {1, 1, 1}}) ==
            (Bbox3f{{0, -1, 0}, {1, 1, 1}}));
    REQUIRE(bbox_union(box, Bbox3f{{0, 0, -1}, {1, 1, 1}}) ==
            (Bbox3f{{0, 0, -1}, {1, 1, 1}}));
}

TEST_CASE("Split box at plane", "[bbox3]") {
    Bbox3i box{{0, 0, 0}, {2, 2, 2}};

    Bbox3i l, r;
    std::tie(l, r) = box.split(0, 1);
    REQUIRE(l == (Bbox3i{{0, 0, 0}, {1, 2, 2}}));
    REQUIRE(r == (Bbox3i{{1, 0, 0}, {2, 2, 2}}));

    std::tie(l, r) = box.split(1, 1);
    REQUIRE(l == (Bbox3i{{0, 0, 0}, {2, 1, 2}}));
    REQUIRE(r == (Bbox3i{{0, 1, 0}, {2, 2, 2}}));

    std::tie(l, r) = box.split(2, 1);
    REQUIRE(l == (Bbox3i{{0, 0, 0}, {2, 2, 1}}));
    REQUIRE(r == (Bbox3i{{0, 0, 1}, {2, 2, 2}}));
}
