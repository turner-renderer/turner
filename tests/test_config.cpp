#include <catch.hpp>
#include "../trace.h"

SCENARIO("Create config from options", "[config]") {

    GIVEN("A a max depth") {
        long max_depth = 2;

    GIVEN("A shadow intensity") {
        float shadow_intensity = 1.0f;

    GIVEN("A number of pixel samples") {
        long num_pixel_samples = 128;

    GIVEN("A number of monte carlo samples") {
        long num_monte_carlo_samples = 2;

    GIVEN("A number of threads") {
        long num_threads = 4;

    GIVEN("A a background color") {
        std::string bg_color = "0 0.3 1.0";

    WHEN("the configuration is created") {
        Configuration config { max_depth
                             , shadow_intensity
                             , num_pixel_samples
                             , num_monte_carlo_samples
                             , num_threads
                             , bg_color
                             };

    THEN("the configuration holds all variables") {
        REQUIRE(config.max_depth == 2);
        REQUIRE(config.shadow_intensity - 1.0f < 0.00001f);
        REQUIRE(config.num_pixel_samples == 128);
        REQUIRE(config.num_monte_carlo_samples == 2);
        REQUIRE(config.num_threads == 4);
        REQUIRE(config.bg_color == Color(0.0f, 0.3f, 1.0f, 1.0f));
    }}}}}}}}
}
