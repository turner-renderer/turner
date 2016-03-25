#include "helper.h"
#include "../lib/monte_carlo.h"
#include <catch.hpp>


TEST_CASE("Test Hemisphere sampling", "[sampling]")
{
    static constexpr int NUM_VECS = 100;
    static constexpr int NUM_SAMPLES = 100;

    Hemisphere hsph;
    Vec vec;
    aiMatrix3x3 trafo;

    for (int i = 0; i < NUM_VECS; ++i) {
        vec = random_vec().Normalize();
        aiMatrix3x3::FromToMatrix(Vec{0, 0, 1}, vec, trafo);

        for (int j = 0; j < NUM_SAMPLES; ++j) {
            auto res = hsph.sample();
            auto sampled_vec = trafo * res.first;
            auto cos_vec = sampled_vec * vec;

            REQUIRE(sampled_vec.Length() - 1.f < 0.0001f);
            REQUIRE(0.f <= cos_vec);
            REQUIRE(cos_vec <= 1.f);
        }
    }
}
