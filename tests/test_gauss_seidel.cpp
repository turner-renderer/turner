#include "../lib/gauss_seidel.h"
#include "../lib/xorshift.h"
#include "helper.h"
#include <catch.hpp>
#include <iostream>

TEST_CASE("Solve identity equation", "[solver]") {
    using MatrixF = math::Matrix<float>;
    using VectorF = math::Vector<float>;

    MatrixF A(2, 2, {1, 0, 0, 1});
    VectorF x(2, {1, 1});
    VectorF b = x;

    auto x_actual = gauss_seidel(A, b, b, 100);
    REQUIRE(x_actual(0) == Approx(x(0)));
    REQUIRE(x_actual(1) == Approx(x(1)));
}

TEST_CASE("Solve random equation", "[solver]") {
    using MatrixF = math::Matrix<float>;
    using VectorF = math::Vector<float>;

    xorshift64star<float> uniform(42);

    MatrixF A(2, 2, {1, uniform(), 0, 1});
    VectorF x(2, {uniform(), uniform()});
    VectorF b(
        2, {A(0, 0) * x(0) + A(0, 1) * x(1), A(1, 0) * x(0) + A(1, 1) * x(1)});

    auto x_actual = gauss_seidel(A, b, b, 100);
    REQUIRE(x_actual(0) == Approx(x(0)));
    REQUIRE(x_actual(1) == Approx(x(1)));
}
