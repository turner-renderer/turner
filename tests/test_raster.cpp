#include "../lib/raster.h"
#include <catch.hpp>

#include <sstream>

TEST_CASE("Test image width", "[raster]") {
    Image img(2, 3);
    REQUIRE(img.width() == 2);
}

TEST_CASE("Test image height", "[raster]") {
    Image img(2, 3);
    REQUIRE(img.height() == 3);
}

TEST_CASE("Test image init values and iteration", "[raster]") {
    Image img(2, 3);
    size_t size = 0;
    for (const auto& x : img) {
        REQUIRE(x == Color());
        size++;
    }
    REQUIRE(size == 6);
}

TEST_CASE("Test image init values and const iteration", "[raster]") {
    const Image const_img(2, 3);
    size_t size = 0;
    for (const auto& x : const_img) {
        REQUIRE(x == Color());
        size++;
    }
    REQUIRE(size == 6);
}

TEST_CASE("Test image color getter/setter values", "[raster]") {
    Image img(2, 3);
    Color white(1, 1, 1, 1);
    img(0, 0) = white;
    REQUIRE(img(0, 0) == white);
}

TEST_CASE("Test image PBM serialization", "[raster]") {
    Image img(2, 2);
    Color white(1, 1, 1, 1);
    img(0, 1) = white;
    std::stringstream ss;
    ss << img;

    constexpr const char* EXPECTED = R"(P3
2 2
255
  0   0   0   0   0   0
255 255 255   0   0   0)";
    REQUIRE(ss.str() == EXPECTED);
}
