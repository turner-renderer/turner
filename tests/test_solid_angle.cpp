#include "../lib/solid_angle.h"
#include "helper.h"
#include <catch.hpp>

TEST_CASE("Solid angle of a randomized triangles", "[solid_angle]") {
    constexpr size_t NUM_TRIANGLES = 1000;
    for (size_t i = 0; i < NUM_TRIANGLES; ++i) {
        const Vec origin = random_vec();
        const Triangle tri = random_triangle();

        float angle = solid_angle(origin, tri);
        float ref_angle_1 = solid_angle_using_dihedral_angles(origin, tri);
        float ref_angle_2 = solid_angle_using_sides_angles(origin, tri);
        if (std::isnan(ref_angle_1) || std::isnan(ref_angle_2)) {
            continue; // skip numerically instable triangles
        }

        REQUIRE(angle == Approx(ref_angle_1).epsilon(0.001));
        REQUIRE(angle == Approx(ref_angle_2).epsilon(0.001));
    }
}
