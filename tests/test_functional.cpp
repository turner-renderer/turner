#include "../lib/functional.h"
#include "helper.h"

#include <catch.hpp>
#include <limits>

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

TEST_CASE("Test hash of a vector, point and normal", "[hash]") {
    std::hash<Point3f> point_hasher;
    std::hash<Vector3f> vector_hasher;
    std::hash<Normal3f> normal_hasher;

    size_t point_res = 0;
    size_t vector_res = 0;
    size_t normal_res = 0;
    for (size_t i = 0; i < 32; ++i) {
        point_res |= point_hasher(random_pt());
        vector_res |= vector_hasher(random_vec());
        normal_res |= normal_hasher(random_normal());
    }

    // all bits are expected to be set, since a good hash function sets a bit
    // uniformly depending on the input
    REQUIRE(static_cast<uint32_t>(point_res) ==
            std::numeric_limits<uint32_t>::max());
    REQUIRE(static_cast<uint32_t>(vector_res) ==
            std::numeric_limits<uint32_t>::max());
    REQUIRE(static_cast<uint32_t>(normal_res) ==
            std::numeric_limits<uint32_t>::max());
}
