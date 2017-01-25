#include "../lib/algorithm.h"
#include "../lib/types.h"
#include "../lib/xorshift.h"
#include <catch.hpp>

#include <algorithm>
#include <numeric>
#include <random>
#include <vector>

namespace {
std::vector<int> shuffled_sequence() {
    std::vector<int> numbers(100, 0);
    std::iota(numbers.begin(), numbers.end(), 0); // numbers [0, 100)

    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(numbers.begin(), numbers.end(), gen);

    return numbers;
}
} // namespace

TEST_CASE("Test min", "[algorithm]") {
    auto numbers = shuffled_sequence();
    int m = ::min(numbers.begin(), numbers.end(), [](int x) { return x; });
    REQUIRE(m == 0);
}

TEST_CASE("Test max", "[algorithm]") {
    auto numbers = shuffled_sequence();
    int m = ::max(numbers.begin(), numbers.end(), [](int x) { return x; });
    REQUIRE(m == 99);
}

TEST_CASE("Test average as float", "[algorithm]") {
    auto numbers = shuffled_sequence();
    float val = average(numbers.begin(), numbers.end(),
                        [](int x) -> float { return x; });
    REQUIRE(val == Approx(49.5));
}

TEST_CASE("Test average as int", "[algorithm]") {
    auto numbers = shuffled_sequence();
    float val =
        average(numbers.begin(), numbers.end(), [](int x) { return x; });
    REQUIRE(val == 49);
}

TEST_CASE("Test average of Colors", "[algorithm]") {
    std::vector<Color> colors = {Color(1, 1, 1, 1), Color(0, 0, 0, 0),
                                 Color(0, 1, 1, 0), Color(1, 1, 1, 1)};
    auto val =
        average(colors.begin(), colors.end(), [](const Color& x) { return x; });
    REQUIRE(val == Color(2.f / 4, 3.f / 4, 3.f / 4, 2.f / 4));
}

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
