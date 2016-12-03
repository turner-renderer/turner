#include "helper.h"
#include "../lib/triangle.h"

#include <catch.hpp>

TEST_CASE("Test Triangle normal", "[triangle]")
{
    static constexpr int NUM_SAMPLES = 100;

    for (int j = 0; j < NUM_SAMPLES; ++j) {
        Triangle triangle = random_triangle();

        Vec normal = triangle.normal;

        // Verify unit lenght of normal.
        float length = normal.Length();
        REQUIRE(length == Approx(1.f));

        // Verify normal is perpendicular to edges of triangle.
        float cos_u = normal * triangle.u.Normalize();
        float cos_v = normal * triangle.v.Normalize();
        REQUIRE(cos_u == Approx(0.f));
        REQUIRE(cos_v == Approx(0.f));
    }
}
