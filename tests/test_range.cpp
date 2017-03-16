#include "../lib/range.h"
#include <catch.hpp>

TEST_CASE("Test range", "[helper]")
{
    // C-style array of prime number.
    int data[4] = { 2, 41, 3, 7};

    auto data_range = make_range(data, 4);

    // Determine invariant.
    int invariant = 0;
    for(auto next : data_range) {
        invariant += next;
    }
    REQUIRE(invariant == 53);
}
