#include "../lib/associative.h"
#include <catch.hpp>

TEST_CASE("Trivial test", "[associative]") {
    AssociativeArray<size_t, int> ar;
    ar.push_back(0);
    ar.push_back(1);

    REQUIRE(ar.size(0) == 0);
    REQUIRE(ar.size(1) == 0);
}
