#include <catch.hpp>
#include "../lib/lambertian.h"

#include <assimp/mesh.h>

SCENARIO("Maximum light", "[light]") {
    GIVEN("A surface normal") {
        Normal3f N(0, 0, 1);

    GIVEN("A surface color") {
        Color surface_color(1, 0, 0, 1);

    GIVEN("A light direction") {
        Vector3f L(0, 0, 1);

    GIVEN("A light intensity") {
        Color I(0.7, 0.5, 0.7, 1);

    WHEN("retrieving the Lambertian reflectance") {
        Color result = lambertian(L, N, surface_color, I);

    THEN("the reflectance is the maximum") {
        REQUIRE(result == Color(0.7, 0, 0, 1));
    }}}}}}
}
