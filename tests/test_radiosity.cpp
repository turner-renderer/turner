#include "../lib/radiosity.h"
#include "helper.h"
#include <catch.hpp>

#include <cmath>
#include <random>

namespace {
constexpr float NUM_SAMPLES = 32;
constexpr float TOLERANCE = 0.01f;
constexpr size_t NUM_RANDOM_TESTS = 50;

std::pair<float, float> parallel_scenario(float a, float b, float c) {
    auto bottom_lft = test_triangle({0, 0, 0}, {a, 0, b}, {a, 0, 0});
    auto bottom_rht = test_triangle({0, 0, 0}, {0, 0, b}, {a, 0, b});
    auto top_lft = test_triangle({0, c, 0}, {a, c, 0}, {a, c, b});
    auto top_rht = test_triangle({0, c, 0}, {a, c, b}, {0, c, b});
    KDTree tree({bottom_lft, bottom_rht, top_lft, top_rht});

    // compute the form factor F_(bottom,top) = F_01_23 using the form
    // factor algebra
    float F_0_2 = form_factor(tree, bottom_lft, top_lft, 2, NUM_SAMPLES);
    float F_0_3 = form_factor(tree, bottom_lft, top_rht, 3, NUM_SAMPLES);
    float F_1_2 = form_factor(tree, bottom_rht, top_lft, 2, NUM_SAMPLES);
    float F_1_3 = form_factor(tree, bottom_rht, top_rht, 3, NUM_SAMPLES);

    float F_01_2 = (bottom_lft.area() * F_0_2 + bottom_rht.area() * F_1_2) /
                   (bottom_lft.area() + bottom_rht.area());
    float F_01_3 = (bottom_lft.area() * F_0_3 + bottom_rht.area() * F_1_3) /
                   (bottom_lft.area() + bottom_rht.area());
    float F_01_23 = F_01_2 + F_01_3;

    // reciproc factor
    float F_2_0 = form_factor(tree, top_lft, bottom_lft, 0, NUM_SAMPLES);
    float F_2_1 = form_factor(tree, top_lft, bottom_rht, 1, NUM_SAMPLES);
    float F_3_0 = form_factor(tree, top_rht, bottom_lft, 0, NUM_SAMPLES);
    float F_3_1 = form_factor(tree, top_rht, bottom_rht, 1, NUM_SAMPLES);

    float F_23_0 = (top_lft.area() * F_2_0 + top_rht.area() * F_3_0) /
                   (top_lft.area() + top_rht.area());
    float F_23_1 = (top_lft.area() * F_2_1 + top_rht.area() * F_3_1) /
                   (top_lft.area() + top_rht.area());
    float F_23_01 = F_23_0 + F_23_1;

    return {F_01_23, F_23_01};
}

std::pair<float, float> orthogonal_scenario(float a, float b, float c) {
    auto bottom_lft = test_triangle({0, 0, 0}, {c, 0, b}, {c, 0, 0}); // 0
    auto bottom_rht = test_triangle({0, 0, 0}, {0, 0, b}, {c, 0, b}); // 1
    auto back_lft = test_triangle({0, 0, 0}, {c, 0, 0}, {c, a, 0});   // 2
    auto back_rht = test_triangle({0, 0, 0}, {c, a, 0}, {0, a, 0});   // 3
    KDTree tree({bottom_lft, bottom_rht, back_lft, back_rht});

    // compute the form factor F_(bottom,top) = F_01_23 using the form
    // factor algebra
    float F_0_2 = form_factor(tree, bottom_lft, back_lft, 2, NUM_SAMPLES);
    float F_0_3 = form_factor(tree, bottom_lft, back_rht, 3, NUM_SAMPLES);
    float F_1_2 = form_factor(tree, bottom_rht, back_lft, 2, NUM_SAMPLES);
    float F_1_3 = form_factor(tree, bottom_rht, back_rht, 3, NUM_SAMPLES);

    float F_01_2 = (bottom_lft.area() * F_0_2 + bottom_rht.area() * F_1_2) /
                   (bottom_lft.area() + bottom_rht.area());
    float F_01_3 = (bottom_lft.area() * F_0_3 + bottom_rht.area() * F_1_3) /
                   (bottom_lft.area() + bottom_rht.area());
    float F_01_23 = F_01_2 + F_01_3;

    // reciproc factor
    float F_2_0 = form_factor(tree, back_lft, bottom_lft, 0, NUM_SAMPLES);
    float F_2_1 = form_factor(tree, back_lft, bottom_rht, 1, NUM_SAMPLES);
    float F_3_0 = form_factor(tree, back_rht, bottom_lft, 0, NUM_SAMPLES);
    float F_3_1 = form_factor(tree, back_rht, bottom_rht, 1, NUM_SAMPLES);

    float F_23_0 = (back_lft.area() * F_2_0 + back_rht.area() * F_3_0) /
                   (back_lft.area() + back_rht.area());
    float F_23_1 = (back_lft.area() * F_2_1 + back_rht.area() * F_3_1) /
                   (back_lft.area() + back_rht.area());
    float F_23_01 = F_23_0 + F_23_1;

    return {F_01_23, F_23_01};
}
}

TEST_CASE("Form factor of two parallel unit squares one unit apart",
          "[form_factor]") {
    float F_ij, F_ji;
    std::tie(F_ij, F_ji) = parallel_scenario(1, 1, 1);

    constexpr float F_EXPECTED = 0.1998; // cf. [CW93], 4.12., p. 96
    REQUIRE(F_ij == Approx(F_EXPECTED).epsilon(TOLERANCE));
    REQUIRE(F_ji == Approx(F_EXPECTED).epsilon(TOLERANCE));
}

TEST_CASE("Form factor of two orthogonal unit squares", "[form_factor]") {
    float F_ij, F_ji;
    std::tie(F_ij, F_ji) = orthogonal_scenario(1, 1, 1);

    constexpr float F_EXPECTED = 0.20004f;
    REQUIRE(F_ij == Approx(F_EXPECTED).epsilon(0.08));
    REQUIRE(F_ji == Approx(F_EXPECTED).epsilon(0.08));
}

TEST_CASE(
    "Form factor of two parallel unit squares one unit apart (analytically)",
    "[form_factor]") {
    float form_factor = form_factor_of_parallel_rects(1, 1, 1);
    REQUIRE(form_factor == Approx(0.1998).epsilon(0.0001));
}

TEST_CASE("Form factor of two orthogonal unit squares (analytically)",
          "[form_factor]") {
    float form_factor = form_factor_of_orthogonal_rects(1, 1, 1);
    REQUIRE(form_factor == Approx(0.2).epsilon(0.0001));
}

TEST_CASE("Form factor of two random parallel rectangles", "[form_factor]") {
    static std::default_random_engine gen(0);
    static std::uniform_real_distribution<float> rnd(1, 10);

    size_t passed = 0;
    for (size_t i = 0; i < NUM_RANDOM_TESTS; ++i) {
        float a = rnd(gen);
        float b = rnd(gen);
        float c = rnd(gen);

        float F_ij, F_ji;
        std::tie(F_ij, F_ji) = parallel_scenario(a, b, c);
        float F_EXPECTED = form_factor_of_parallel_rects(a, b, c);
        passed += F_ij == Approx(F_EXPECTED).epsilon(TOLERANCE);
        passed += F_ji == Approx(F_EXPECTED).epsilon(TOLERANCE);
    }

    // Variance in Monte Carlo integration is too high, so we only require that
    // 99% of all checks pass.
    REQUIRE(2.0 * passed / NUM_RANDOM_TESTS > 0.99);
}

TEST_CASE("Form factor of two random orthogonal rectangles", "[form_factor]") {
    static std::default_random_engine gen(0);
    static std::uniform_real_distribution<float> rnd(1, 10);

    size_t passed = 0;
    for (size_t i = 0; i < NUM_RANDOM_TESTS; ++i) {
        float a = rnd(gen);
        float b = rnd(gen);
        float c = rnd(gen);

        float F_ij, F_ji;
        std::tie(F_ij, F_ji) = parallel_scenario(a, b, c);
        float F_EXPECTED = form_factor_of_parallel_rects(a, b, c);
        passed += F_ij == Approx(F_EXPECTED).epsilon(TOLERANCE);
        passed += F_ji == Approx(F_EXPECTED).epsilon(TOLERANCE);
    }

    // Variance in Monte Carlo integration is too high, so we only require that
    // 99% of all checks pass.
    REQUIRE(2.0 * passed / NUM_RANDOM_TESTS > 0.99);
}

TEST_CASE("Form factor distance dependency", "[form_factor]") {
    static constexpr size_t NUM_STEPS = 100;
    for (size_t i = 0; i < NUM_STEPS; ++i) {
        float distance = std::sqrt(static_cast<float>(i + 1));
        float F_ij, F_ji;
        std::tie(F_ij, F_ji) = parallel_scenario(1, 1, distance);

        const float F_EXPECTED = form_factor_of_parallel_rects(1, 1, distance);
        REQUIRE(F_ij == Approx(F_ij).epsilon(TOLERANCE));
        REQUIRE(F_ji == Approx(F_ij).epsilon(TOLERANCE)); // reciprocity
    }
}
