#include "../lib/geometry.h"
#include <catch.hpp>

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
