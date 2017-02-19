#include "../lib/functional.h"

#include <catch.hpp>
#include <limits>
#include <random>

TEST_CASE("Test hash of a pair", "[hash]") {
    std::random_device rd;
    std::uniform_int_distribution<uint64_t> dist(
        0, std::numeric_limits<uint64_t>::max());
    std::hash<std::pair<uint64_t, uint64_t>> hasher;

    size_t res = 0;
    for (size_t i = 0; i < 64; ++i) {
        std::pair<uint64_t, uint64_t> p = {dist(rd), dist(rd)};
        res |= hasher(p);
    }

    // all bits are expected to be set, since a good hash function sets a bit
    // uniformly depending on the input
    REQUIRE(res == std::numeric_limits<size_t>::max());
}
