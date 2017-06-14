#include "../src/geometry.h"
#include <catch.hpp>
#include <iostream>

using namespace turner;

TEST_CASE("Point2 default constructor", "[point2]") {
    Point2f p;
    REQUIRE(p.x == 0);
    REQUIRE(p.y == 0);
}

TEST_CASE("Point2 constructor", "[point2]") {
    Point2f p(-1, 1);
    REQUIRE(p.x == -1);
    REQUIRE(p.y == 1);
}

TEST_CASE("Point2 addition operators", "[point2]") {

    Point2f p1(-1, 1);
    p1 += Point2f(2, 1);
    REQUIRE(p1 == Point2f(1, 2));

    p1 = Point2f(-1, 1);
    p1 += Vector2f(2, 1);
    REQUIRE(p1 == Point2f(1, 2));

    Point2f p3 = p1 + Point2f(2, 1);
    REQUIRE(p3 == Point2f(3, 3));
    REQUIRE(p1 == Point2f(1, 2));

    Point2f p4 = p1 + Vector2f(1, 2);
    REQUIRE(p4 == Point2f(2, 4));
    REQUIRE(p1 == Point2f(1, 2));
}

TEST_CASE("Point2 subtraction operators", "[point2]") {

    Point2f p1(-1, 1);
    p1 -= Vector2f(2, 1);
    REQUIRE(p1 == Point2f(-3, 0));

    Point2f p2 = p1 - Vector2f(1, 2);
    REQUIRE(p2 == Point2f(-4, -2));
    REQUIRE(p1 == Point2f(-3, 0));
}

TEST_CASE("Point2 multiplication operators", "[point2]") {
    Point2f p1(-1, 1.6);
    Point2f p2 = p1 * 0.5;
    REQUIRE(p2 == Point2f(-0.5, 0.8));

    p2 *= 0.2;
    REQUIRE(p2.x == Approx(-0.1));
    REQUIRE(p2.y == Approx(0.16));
}

TEST_CASE("Point2 division operators", "[point2]") {
    Point2f p1(-1, 1.6);
    Point2f p2 = p1 / 2;
    REQUIRE(p2 == Point2f(-0.5, 0.8));

    p2 /= 5;
    REQUIRE(p2.x == Approx(-0.1));
    REQUIRE(p2.y == Approx(0.16));
}

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
