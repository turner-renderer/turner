#include "../lib/filter.h"
#include <catch.hpp>

using namespace turner;

TEST_CASE("Filter smoke test", "[filter]") { BaseFilter f(Vector2f(1, 2)); }
