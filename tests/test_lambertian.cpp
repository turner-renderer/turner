#include "catch.hpp"
#include "../lib/lambertian.h"

#include <assimp/mesh.h>

SCENARIO("Maximum light", "[light]") {
    GIVEN("A surface normal") {
        aiVector3D N(0, 0, 1);

        GIVEN("A surface color") {
            aiColor4D surface_color(1, 0, 0, 1);

            GIVEN("A light direction") {
                aiVector3D L(0, 0, 1);

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
