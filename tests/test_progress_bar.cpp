#include "../lib/progress_bar.h"
#include <catch.hpp>
#include <iostream>
#include <sstream>

TEST_CASE("Progress bar prints 50%", "[output]") {
    std::stringstream out;
    const auto progress_bar = ProgressBar(out, "Test", 30);

    progress_bar.update(15);
    REQUIRE(out.str() == "\rTest                ■■■■■■■■■■□□□□□□□□□□  50.00%");
}

TEST_CASE("Progress bar prints 66%", "[output]") {
    std::stringstream out;
    const auto progress_bar = ProgressBar(out, "Test", 30);

    progress_bar.update(20);
    REQUIRE(out.str() == "\rTest                ■■■■■■■■■■■■■□□□□□□□  66.67%");
}

TEST_CASE("Progress bar prints over 100%", "[output]") {
    std::stringstream out;
    const auto progress_bar = ProgressBar(out, "Test", 30);

    progress_bar.update(45);
    REQUIRE(out.str() == "\rTest                ■■■■■■■■■■■■■■■■■■■■ 150.00%");
}
