#include "../lib/output.h"
#include "../lib/runtime.h"
#include "../lib/triangle.h"
#include "../lib/types.h"
#include "helper.h"
#include <catch.hpp>
#include <cereal/archives/portable_binary.hpp>

TEST_CASE("Test bbox of triangle", "[triangle]") {
    REQUIRE(test_triangle({0, 0, 0}, {0, 0, 1}, {0, 1, 1}).bbox() ==
            (Bbox3f{{0, 0, 0}, {0, 1, 1}}));
    REQUIRE(test_triangle({-1, 0, 0}, {0, 0, 1}, {0, 1, 1}).bbox() ==
            (Bbox3f{{-1, 0, 0}, {0, 1, 1}}));
    REQUIRE(test_triangle({0, 0, 0}, {0, -1, 1}, {0, 1, 1}).bbox() ==
            (Bbox3f{{0, -1, 0}, {0, 1, 1}}));
    REQUIRE(test_triangle({0, 0, 0}, {0, 0, -1}, {0, 1, 1}).bbox() ==
            (Bbox3f{{0, 0, -1}, {0, 1, 1}}));
    REQUIRE(test_triangle({0, 0, 0}, {0, 0, 1}, {2, 1, 1}).bbox() ==
            (Bbox3f{{0, 0, 0}, {2, 1, 1}}));
}

TEST_CASE("Test is_planar of triangle", "[triangle]") {
    for (auto ax : AXES) {
        Point3f pts[3] = {random_pt(), random_pt(), random_pt()};
        for (int i = 0; i < 3; ++i) {
            pts[i][static_cast<int>(ax)] = 0.f;
        }

        Triangle triangle = test_triangle(pts[0], pts[1], pts[2]);
        REQUIRE(triangle.is_planar(ax));
    }
}

TEST_CASE("Test serialization of triangle", "[triangle]") {
    Triangle tri_out = random_triangle();

    // Serialize triangle
    std::ostringstream os;
    {
        cereal::PortableBinaryOutputArchive oarchive(os);
        oarchive(tri_out);
    }

    // Deserialize triangle again
    Triangle tri_in;
    std::istringstream is(os.str());
    {
        cereal::PortableBinaryInputArchive iarchive(is);
        iarchive(tri_in);
    }

    REQUIRE(tri_in.vertices[0] == tri_out.vertices[0]);
    REQUIRE(tri_in.vertices[1] == tri_out.vertices[1]);
    REQUIRE(tri_in.vertices[2] == tri_out.vertices[2]);
}

TEST_CASE("Test clamping", "[clamping]") {
    REQUIRE(clamp(-1, 0, 2) == 0);
    REQUIRE(clamp(0, 0, 2) == 0);
    REQUIRE(clamp(1, 0, 2) == 1);
    REQUIRE(clamp(2, 0, 2) == 2);
    REQUIRE(clamp(3, 0, 2) == 2);

    REQUIRE(clamp(-1.f, 0.f, 1.f) == 0.f);
    REQUIRE(clamp(0.f, 0.f, 1.f) == 0.f);
    REQUIRE(clamp(0.5f, 0.f, 1.f) == 0.5f);
    REQUIRE(clamp(1.f, 0.f, 1.f) == 1.f);
    REQUIRE(clamp(2.f, 0.f, 1.f) == 1.f);
}
