#include "../lib/solid_angle.h"
#include "helper.h"
#include <catch.hpp>

RadiosityMesh::Point random_point() {
    static std::default_random_engine gen(0);
    static std::uniform_real_distribution<float> rnd(-10.f, 10.f);
    return {rnd(gen), rnd(gen), rnd(gen)};
};

TEST_CASE("Solid angle of a randomized triangles", "[solid_angle]") {
    constexpr size_t NUM_TRIANGLES = 1000;
    for (size_t i = 0; i < NUM_TRIANGLES; ++i) {
        const auto O = random_point();
        const auto A = random_point();
        const auto B = random_point();
        const auto C = random_point();

        float angle = solid_angle(O, A, B, C);
        float ref_angle_1 = solid_angle_using_dihedral_angles(O, A, B, C);
        float ref_angle_2 = solid_angle_using_sides_angles(O, A, B, C);
        if (std::isnan(ref_angle_1) || std::isnan(ref_angle_2)) {
            continue; // skip numerically instable triangles
        }

        REQUIRE(angle == Approx(ref_angle_1).epsilon(0.001));
        REQUIRE(angle == Approx(ref_angle_2).epsilon(0.001));
    }
}
