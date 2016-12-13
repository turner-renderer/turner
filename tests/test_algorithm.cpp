#include "../lib/algorithm.h"
#include "../lib/types.h"
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
    float val =
        avg(numbers.begin(), numbers.end(), [](int x) -> float { return x; });
    REQUIRE(val == Approx(49.5));
}

TEST_CASE("Test average as int", "[algorithm]") {
    auto numbers = shuffled_sequence();
    float val = avg(numbers.begin(), numbers.end(), [](int x) { return x; });
    REQUIRE(val == 49);
}

TEST_CASE("Test average of Colors", "[algorithm]") {
    std::vector<Color> colors = {Color(1, 1, 1, 1), Color(0, 0, 0, 0),
                                 Color(0, 1, 1, 0), Color(1, 1, 1, 1)};
    auto val =
        avg(colors.begin(), colors.end(), [](const Color& x) { return x; });
    REQUIRE(val == Color(2.f / 4, 3.f / 4, 3.f / 4, 2.f / 4));
}
