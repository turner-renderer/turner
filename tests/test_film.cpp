#include "../lib/film.h"
#include "../lib/filter.h"
#include "../lib/spectrum.h"
#include <catch.hpp>
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

TEST_CASE("Film tile test", "[film]") {
    BoxFilter filter({0, 0});
    Point2i resolution(600, 480);
    Bbox2f cropped_bbox({0, 0}, {0.01, 0.01});
    Film<BoxFilter> film(resolution, cropped_bbox, filter, 1);
    auto tile = film.get_tile<Spectrum>({{1, 1}, {4, 4}});
    // REQUIRE(tile.get_pixel_bbox() == Bbox2i({1, 1}, {4, 4}));

    tile.add_sample({2.5, 2.5}, Spectrum::from_rgb(1, 1, 1));
    film.merge_tile(tile);

    std::ostringstream oss;
    oss << film;
    static const char* expected = 1 + R"_(
P3
6 5
255
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
  0   0   0   0   0   0 255 255 255   0   0   0   0   0   0   0   0   0
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
  0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0   0
)_";
    REQUIRE(oss.str() == expected);
}
