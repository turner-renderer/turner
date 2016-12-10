#include "helper.h"
#include "../lib/triangle.h"
#include "../lib/hierarchical.h"

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

TEST_CASE("Test subdivision of triangle", "[triangle]")
{
    static constexpr int NUM_SAMPLES = 100;

    for (int j = 0; j < NUM_SAMPLES; ++j) {
        Triangle triangle = random_triangle();

        auto triangles = subdivide4(triangle);

        // Verify that all triangles have the same size.
        REQUIRE(triangles[0].area() == Approx(triangles[1].area()));
        REQUIRE(triangles[1].area() == Approx(triangles[2].area()));
        REQUIRE(triangles[2].area() == Approx(triangles[3].area()));
        REQUIRE(triangles[3].area() == Approx(triangles[0].area()));

        // Verify sum of sub triangles to be full area.
        float summed_area = triangles[0].area() + triangles[1].area() +
            triangles[2].area() + triangles[3].area();
        REQUIRE(summed_area == Approx(triangle.area()));
    }
}
