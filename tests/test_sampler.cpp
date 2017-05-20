#include "../lib/sampler/stratified.h"
#include <catch.hpp>

TEST_CASE("smoke", "[sampler]") { StratifiedSampler sampler(1, 1, true, 1); }
