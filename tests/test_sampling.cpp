#include "helper.h"
#include "../lib/output.h"
#include "../lib/sampling.h"
#include "../lib/intersection.h"

#include <catch.hpp>

namespace {
constexpr float TOLERANCE = 0.01f;
}

TEST_CASE("Test Hemisphere sampling", "[sampling]")
{
    static constexpr int NUM_VECS = 100;
    static constexpr int NUM_SAMPLES = 100;

    Vec vec;
    aiMatrix3x3 trafo;

    for (int i = 0; i < NUM_VECS; ++i) {
        vec = random_vec().Normalize();
        aiMatrix3x3::FromToMatrix(Vec{0, 0, 1}, vec, trafo);

        for (int j = 0; j < NUM_SAMPLES; ++j) {
            auto res = sampling::hemisphere();
            auto sampled_vec = trafo * res.first;
            auto cos_vec = sampled_vec * vec;

            REQUIRE(cos_vec == Approx(res.second).epsilon(TOLERANCE));

            REQUIRE(sampled_vec.Length() == Approx(1.f).epsilon(TOLERANCE));
            REQUIRE(0.f <= cos_vec);
            REQUIRE(cos_vec <= 1.f);
        }
    }
}

TEST_CASE("Test Triangle sampling", "[sampling]")
{
    static constexpr int NUM_SAMPLES = 100;
    float r, s, t;

    for (int j = 0; j < NUM_SAMPLES; ++j) {
        Triangle triangle = random_triangle();

        Vec sample = sampling::triangle(triangle);
        float length = sample.Length();

        // Verify that point is on triangle.
        bool intersect = intersect_ray_triangle(
            Ray{Vec{0, 0, 0}, sample.Normalize()}, triangle, r, s, t);
        REQUIRE(intersect);
        REQUIRE(length == Approx(r).epsilon(TOLERANCE));
    }
}
