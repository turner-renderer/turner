#include "helper.h"
#include "../lib/hierarchical.h"

#include <catch.hpp>

TEST_CASE("Test subdivision of triangle", "[triangle]")
{
    static constexpr int NUM_SAMPLES = 100;

    for (int j = 0; j < NUM_SAMPLES; ++j) {
        Triangle triangle = random_triangle();

        auto triangles = subdivide4(triangle);

        // Verify that all triangles have the same size
        REQUIRE(triangles[0].area() == Approx(triangles[1].area()));
        REQUIRE(triangles[1].area() == Approx(triangles[2].area()));
        REQUIRE(triangles[2].area() == Approx(triangles[3].area()));
        REQUIRE(triangles[3].area() == Approx(triangles[0].area()));
    }
}
