#include "../lib/film.h"
#include "../lib/filter.h"
#include <catch.hpp>
#include <iostream>
#include <sstream>

using namespace turner;

TEST_CASE("Film smoke test", "[film]") {
    GaussianFilter filter({1, 1}, 0.1);
    Point2i resolution(600, 480);
    Bbox2f cropped_bbox({0, 0}, {0.01, 0.01});
    Film<GaussianFilter> film(resolution, cropped_bbox, filter, 1);

    std::ostringstream oss;
    oss << film;
    static const char* expected = 1 + R"_(
P3
6 5
255
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
)_";
    REQUIRE(oss.str() == expected);
}
