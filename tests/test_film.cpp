#include "../lib/film.h"
#include "../lib/filter.h"
#include <catch.hpp>
#include <iostream>

using namespace turner;

TEST_CASE("Film smoke test", "[film]") {
    GaussianFilter filter({1, 1}, 0.1);
    Film<GaussianFilter> film({0, 600}, {{0.1, 0.1}, {0.9, 0.9}}, filter, 1);
    std::cerr << film.resolution.x << std::endl;
}
