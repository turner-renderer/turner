#include "../lib/algorithm.h"
#include <catch.hpp>

#include <algorithm>
#include <numeric>
#include <random>
#include <vector>

namespace {
std::vector<int> random_sequence() {
    std::vector<int> numbers(100, 0);
    std::iota(numbers.begin(), numbers.end(), 0); // numbers [0, 100)

    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(numbers.begin(), numbers.end(), gen);

    return numbers;
}
} // namespace

TEST_CASE("Test min", "[algorithm]") {
    auto numbers = random_sequence();
    int m = ::min(numbers.begin(), numbers.end(), [](int x) { return x; });
    REQUIRE(m == 0);
}

TEST_CASE("Test max", "[algorithm]") {
    auto numbers = random_sequence();
    int m = ::max(numbers.begin(), numbers.end(), [](int x) { return x; });
    REQUIRE(m == 99);
}
