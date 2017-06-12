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

    Vector3f vec;
    aiMatrix3x3 trafo;

    for (int i = 0; i < NUM_VECS; ++i) {
        vec = normalize(random_vec());
        aiMatrix3x3::FromToMatrix(aiVector3D{0, 0, 1}, {vec.x, vec.y, vec.z},
                                  trafo);

        for (int j = 0; j < NUM_SAMPLES; ++j) {
            auto res = sampling::hemisphere();
            aiVector3D ai_res = {res.first.x, res.first.y, res.first.z};
            aiVector3D ai_sampled_vec = trafo * ai_res;
            Vector3f sampled_vec(ai_sampled_vec.x, ai_sampled_vec.y,
                                 ai_sampled_vec.z);
            float cos_vec = dot(sampled_vec, vec);

            REQUIRE(cos_vec == Approx(res.second).epsilon(TOLERANCE));

            REQUIRE(sampled_vec.length() == Approx(1.f).epsilon(TOLERANCE));
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
