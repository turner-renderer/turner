#include "../lib/lambertian.h"
#include <catch.hpp>

SCENARIO("Maximum light", "[light]") {
    GIVEN("A surface normal") {
        Normal3f N(0, 0, 1);

        GIVEN("A surface color") {
            aiColor4D surface_color(1, 0, 0, 1);

            GIVEN("A light direction") {
                Vector3f L(0, 0, 1);

                GIVEN("A light intensity") {
                    aiColor4D I(0.7, 0.5, 0.7, 1);

                    WHEN("retrieving the Lambertian reflectance") {
                        aiColor4D result = lambertian(L, N, surface_color, I);

                        THEN("the reflectance is the maximum") {
                            REQUIRE(result == aiColor4D(0.7, 0, 0, 1));
                        }
                    }
                }
            }
        }
    }
}
