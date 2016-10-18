#include "../lib/effects.h"
#include <catch.hpp>

SCENARIO("Zero exposure", "[effects]") {
    GIVEN("A light") {
        float light = 0.5f;
    GIVEN("Zero exposure") {
        float value = 0;
    WHEN("calculating light's exposure") {
        float exposed_light = exposure(light, value);
    THEN("the exposed light is dark") {
        REQUIRE(exposed_light == 0);
    }}}}
}

SCENARIO("Low exposure", "[effects]") {
    GIVEN("A light") {
        float light = 0.5f;
    GIVEN("A low exposure") {
        float value = 1;
    WHEN("calculating light's exposure") {
        float exposed_light = exposure(light, value);
    THEN("the exposed light is darker") {
        REQUIRE(exposed_light < light);
    }}}}
}

SCENARIO("High exposure", "[effects]") {
    GIVEN("A light") {
        float light = 0.5f;
    GIVEN("A high exposure") {
        float value = 10;
    WHEN("calculating light's exposure") {
        float exposed_light = exposure(light, value);
    THEN("the exposed light is brighter") {
        REQUIRE(light < exposed_light);
    }}}}
}

SCENARIO("Zero exposure applied to color", "[effects]") {
    GIVEN("A color") {
        Color color{0.5f, 0.5f, 0.5f, 1};
    GIVEN("Zero exposure") {
        float value = 0;
    WHEN("calculating color's exposure") {
        Color exposed_color = exposure(color, value);
    THEN("the exposed color is zero except for alpha channel") {
        REQUIRE(exposed_color == (Color{0, 0, 0, 1}));
    }}}}
}

SCENARIO("Default gamma correction", "[effects]") {
    GIVEN("A light") {
        float light = 0.5f;
    WHEN("calculating light's gamma correction") {
        float correct_light = gamma(light);
    THEN("the gamma-corrected light is brighter") {
        REQUIRE(light < correct_light);
    }}}
}

SCENARIO("Default gamma correction applied to color", "[effects]") {
    GIVEN("A color") {
        Color color{0.5f, 0.5f, 0.5f, 1};
    WHEN("calculating colors's gamma correction") {
        Color correct_color = gamma(color);
    THEN("the gamma-corrected color is brighter") {
        REQUIRE(color < correct_color);
    THEN("and alpha channel has not changed") {
        REQUIRE(color.a == correct_color.a);
    }}}}
}
