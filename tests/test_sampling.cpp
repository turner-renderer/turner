#include "../lib/intersection.h"
#include "../lib/output.h"
#include "../lib/sampling.h"
#include "helper.h"

#include <catch.hpp>

namespace {
constexpr float TOLERANCE = 0.01f;
}

TEST_CASE("Test Hemisphere sampling", "[sampling]") {
    static constexpr int NUM_VECS = 100;
    static constexpr int NUM_SAMPLES = 100;

    aiMatrix3x3 trafo;

    for (int i = 0; i < NUM_VECS; ++i) {
        Vector3f v = normalize(random_vec());
        aiMatrix3x3::FromToMatrix({0, 0, 1}, {v.x, v.y, v.z}, trafo);

        for (int j = 0; j < NUM_SAMPLES; ++j) {
            auto res = sampling::hemisphere();
            auto sampled_v_ai =
                trafo * aiVector3D{res.first.x, res.first.y, res.first.z};
            Vector3f sampled_v{sampled_v_ai.x, sampled_v_ai.y, sampled_v_ai.z};
            auto cos_vec = dot(sampled_v, v);

            REQUIRE(cos_vec == Approx(res.second).epsilon(TOLERANCE));

            REQUIRE(sampled_v.length() == Approx(1.f).epsilon(TOLERANCE));
            REQUIRE(0.f <= cos_vec);
            REQUIRE(cos_vec <= 1.f);
        }
    }
}

TEST_CASE("Test Triangle sampling", "[sampling]") {
    static constexpr int NUM_SAMPLES = 100;
    float r, s, t;

    for (int j = 0; j < NUM_SAMPLES; ++j) {
        Triangle triangle = random_triangle();

        Vector3f sample = Vector3f(sampling::triangle(triangle));
        float length = sample.length();

        // Verify that point is on triangle.
        bool intersect = intersect_ray_triangle(
            Ray{{0, 0, 0}, normalize(sample)}, triangle, r, s, t);
        REQUIRE(intersect);
        REQUIRE(length == Approx(r).epsilon(TOLERANCE));
    }
}
