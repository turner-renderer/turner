#include "../lib/radiosity.h"
#include "helper.h"
#include <catch.hpp>

namespace {
constexpr float NUM_SAMPLES = 32;
constexpr float TOLERANCE = 0.01f;
}

TEST_CASE("Form factor of two parallel unit squares one unit apart",
          "[form_factor]") {
    // two triangulated parallel unit squares one unit distance apart,
    // cf. [CW93], Figure 4.22.
    auto bottom_left =
        test_triangle({0.5, -0.5, 0}, {0.5, 0.5, 0}, {-0.5, 0.5, 0}, {0, 0, 1},
                      {0, 0, 1}, {0, 0, 1}); // 0
    auto bottom_right =
        test_triangle({0.5, -0.5, 0}, {-0.5, 0.5, 0}, {-0.5, -0.5, 0},
                      {0, 0, 1}, {0, 0, 1}, {0, 0, 1}); // 1
    auto top_left =
        test_triangle({-0.5f, 0.5f, 1}, {0.5, 0.5, 1}, {0.5, -0.5, 1},
                      {0, 0, -1}, {0, 0, -1}, {0, 0, -1}); // 2
    auto top_right =
        test_triangle({-0.5f, 0.5f, 1}, {0.5, -0.5, 1}, {-0.5, -0.5, 1},
                      {0, 0, -1}, {0, 0, -1}, {0, 0, -1}); // 3
    KDTree tree({bottom_left, bottom_right, top_left, top_right});

    // compute the form factor F_(bottom,top) = F_01_23 using the form factor
    // algebra
    float F_0_2 = form_factor(tree, 0, 2, NUM_SAMPLES);
    float F_0_3 = form_factor(tree, 0, 3, NUM_SAMPLES);
    float F_1_2 = form_factor(tree, 1, 2, NUM_SAMPLES);
    float F_1_3 = form_factor(tree, 1, 3, NUM_SAMPLES);

    float F_01_2 = (bottom_left.area() * F_0_2 + bottom_right.area() * F_1_2) /
                   (bottom_left.area() + bottom_right.area());
    float F_01_3 = (bottom_left.area() * F_0_3 + bottom_right.area() * F_1_3) /
                   (bottom_left.area() + bottom_right.area());
    float F_01_23 = F_01_2 + F_01_3;

    constexpr float F_EXPECTED = 0.1998; // cf. [CW93], 4.12.
    REQUIRE(F_01_23 == Approx(F_EXPECTED).epsilon(TOLERANCE));

    // symmetry test
    float F_2_0 = form_factor(tree, 2, 0, NUM_SAMPLES);
    float F_2_1 = form_factor(tree, 2, 1, NUM_SAMPLES);
    float F_3_0 = form_factor(tree, 3, 0, NUM_SAMPLES);
    float F_3_1 = form_factor(tree, 3, 1, NUM_SAMPLES);

    float F_23_0 = (top_left.area() * F_2_0 + top_right.area() * F_3_0) /
                   (top_left.area() + top_right.area());
    float F_23_1 = (top_left.area() * F_2_1 + top_right.area() * F_3_1) /
                   (top_left.area() + top_right.area());
    float F_23_01 = F_23_0 + F_23_1;

    REQUIRE(F_23_01 == Approx(F_EXPECTED).epsilon(TOLERANCE));
}

TEST_CASE("Form factor of two parallel unit squares one unit apart (Version 2)",
          "[form_factor]") {
    // two triangulated parallel unit squares one unit distance apart,
    // cf. [CW93], Figure 4.22.
    auto bottom_left =
        test_triangle({0.5, -0.5, 0}, {0.5, 0.5, 0}, {-0.5, 0.5, 0}, {0, 0, 1},
                      {0, 0, 1}, {0, 0, 1}); // 0
    auto bottom_right =
        test_triangle({0.5, -0.5, 0}, {-0.5, 0.5, 0}, {-0.5, -0.5, 0},
                      {0, 0, 1}, {0, 0, 1}, {0, 0, 1}); // 1
    auto top_left =
        test_triangle({-0.5f, 0.5f, 1}, {0.5, 0.5, 1}, {0.5, -0.5, 1},
                      {0, 0, -1}, {0, 0, -1}, {0, 0, -1}); // 2
    auto top_right =
        test_triangle({-0.5f, 0.5f, 1}, {0.5, -0.5, 1}, {-0.5, -0.5, 1},
                      {0, 0, -1}, {0, 0, -1}, {0, 0, -1}); // 3
    KDTree tree({bottom_left, bottom_right, top_left, top_right});

    // compute the form factor F_(bottom,top) = F_01_23 using the form factor
    // algebra
    float F_0_2 = form_factor(tree, bottom_left, top_left, 2, NUM_SAMPLES);
    float F_0_3 = form_factor(tree, bottom_left, top_right, 3, NUM_SAMPLES);
    float F_1_2 = form_factor(tree, bottom_right, top_left, 2, NUM_SAMPLES);
    float F_1_3 = form_factor(tree, bottom_right, top_right, 3, NUM_SAMPLES);

    float F_01_2 = (bottom_left.area() * F_0_2 + bottom_right.area() * F_1_2) /
                   (bottom_left.area() + bottom_right.area());
    float F_01_3 = (bottom_left.area() * F_0_3 + bottom_right.area() * F_1_3) /
                   (bottom_left.area() + bottom_right.area());
    float F_01_23 = F_01_2 + F_01_3;

    constexpr float F_EXPECTED = 0.1998; // cf. [CW93], 4.12.
    REQUIRE(F_01_23 == Approx(F_EXPECTED).epsilon(TOLERANCE));

    // symmetry test
    float F_2_0 = form_factor(tree, top_left, bottom_left, 0, NUM_SAMPLES);
    float F_2_1 = form_factor(tree, top_left, bottom_right, 1, NUM_SAMPLES);
    float F_3_0 = form_factor(tree, top_right, bottom_left, 0, NUM_SAMPLES);
    float F_3_1 = form_factor(tree, top_right, bottom_right, 1, NUM_SAMPLES);

    float F_23_0 = (top_left.area() * F_2_0 + top_right.area() * F_3_0) /
                   (top_left.area() + top_right.area());
    float F_23_1 = (top_left.area() * F_2_1 + top_right.area() * F_3_1) /
                   (top_left.area() + top_right.area());
    float F_23_01 = F_23_0 + F_23_1;

    REQUIRE(F_23_01 == Approx(F_EXPECTED).epsilon(TOLERANCE));
}
