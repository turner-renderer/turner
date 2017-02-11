#include "../config.h"

#include <catch.hpp>
#include <docopt/docopt.h>

#include <sstream>

namespace raycaster {
#include "../raycaster.h"
}
namespace raytracer {
#include "../raytracer.h"
}
namespace pathtracer {
#include "../pathtracer.h"
}
namespace radiosity {
#include "../radiosity.h"
}

void test_common_config(const char* usage) {
    const char* argv[] = {"./exec",      "-w",
                          "100",         "-a",
                          "0.5",         "--background",
                          "0.5 0.5 0.5", "-t",
                          "2",           "--inverse-gamma",
                          "1",           "--no-gamma-correction",
                          "--exposure",  "1.5",
                          "file"};

    std::map<std::string, docopt::value> args =
        docopt::docopt(usage, {argv + 1, argv + 15});
    Config conf = Config::from_docopt(args);

    REQUIRE(conf.width == 100);
    REQUIRE(conf.aspect == 0.5);
    REQUIRE(conf.bg_color == Color(0.5, 0.5, 0.5, 1));
    REQUIRE(conf.num_threads == 2);
    REQUIRE(conf.inverse_gamma == 1);
    REQUIRE(conf.gamma_correction_enabled == false);
    REQUIRE(conf.exposure == 1.5);
    REQUIRE(conf.filename == "file");
}

TEST_CASE("Create config from raycaster USAGE", "[config]") {
    test_common_config(raycaster::USAGE);

    const char* argv[] = {"./exec", "--max-visibility", "4.5", "file"};
    std::map<std::string, docopt::value> args =
        docopt::docopt(raycaster::USAGE, {argv + 1, argv + 4});
    auto conf = TracerConfig::from_docopt(args);

    REQUIRE(conf.max_visibility == 4.5);

    std::ostringstream os;
    os << conf;
    REQUIRE(os.str().size() > 0);
}

TEST_CASE("Create config from raytracer USAGE", "[config]") {
    test_common_config(raytracer::USAGE);

    const char* argv[] = {"./exec", "-d", "42", "--shadow", "0.7", "file"};
    std::map<std::string, docopt::value> args =
        docopt::docopt(raytracer::USAGE, {argv + 1, argv + 6});
    auto conf = TracerConfig::from_docopt(args);

    REQUIRE(conf.max_recursion_depth == 42);
    REQUIRE(conf.shadow_intensity == 0.7f);

    std::ostringstream os;
    os << conf;
    REQUIRE(os.str().size() > 0);
}

TEST_CASE("Create config from pathtracer USAGE", "[config]") {
    test_common_config(pathtracer::USAGE);

    const char* argv[] = {"./exec", "-d", "42", "-p", "42", "-m", "42", "file"};
    std::map<std::string, docopt::value> args =
        docopt::docopt(pathtracer::USAGE, {argv + 1, argv + 8});
    auto conf = TracerConfig::from_docopt(args);

    REQUIRE(conf.max_recursion_depth == 42);
    REQUIRE(conf.num_pixel_samples == 42);
    REQUIRE(conf.num_monte_carlo_samples == 42);

    std::ostringstream os;
    os << conf;
    REQUIRE(os.str().size() > 0);
}

TEST_CASE("Create common config from radiosity USAGE", "[config]") {
    const char* argv[] = {"./exec",
                          "exact",
                          "-w",
                          "100",
                          "-a",
                          "0.5",
                          "--background",
                          "0.5 0.5 0.5",
                          "-t",
                          "2",
                          "--inverse-gamma",
                          "1",
                          "--no-gamma-correction",
                          "--exposure",
                          "1.5",
                          "file"};

    std::map<std::string, docopt::value> args =
        docopt::docopt(radiosity::USAGE, {argv + 1, argv + 16});
    Config conf = Config::from_docopt(args);

    REQUIRE(conf.width == 100);
    REQUIRE(conf.aspect == 0.5);
    REQUIRE(conf.bg_color == Color(0.5, 0.5, 0.5, 1));
    REQUIRE(conf.num_threads == 2);
    REQUIRE(conf.inverse_gamma == 1);
    REQUIRE(conf.gamma_correction_enabled == false);
    REQUIRE(conf.exposure == 1.5);
    REQUIRE(conf.filename == "file");
}

TEST_CASE("Create config from radiosity USAGE", "[config]") {
    const char* argv[] = {"./exec",
                          "exact",
                          "--form-factor-eps",
                          "0.1",
                          "--rad-shoot-eps",
                          "0.1",
                          "--max-subdivisions",
                          "42",
                          "--max-iterations",
                          "42",
                          "file"};

    std::map<std::string, docopt::value> args =
        docopt::docopt(radiosity::USAGE, {argv + 1, argv + 11});
    auto conf = RadiosityConfig::from_docopt(args);

    REQUIRE(conf.mode == RadiosityConfig::EXACT);
    REQUIRE(conf.F_eps == 0.1f);
    REQUIRE(conf.BF_eps == 0.1f);
    REQUIRE(conf.max_subdivisions == 42);
    REQUIRE(conf.max_iterations == 42);

    // test standard values of flags
    REQUIRE(!conf.gouraud_enabled);
    REQUIRE(conf.mesh == RadiosityConfig::NO_MESH);
    REQUIRE(!conf.links_enabled);
    REQUIRE(!conf.exact_hierarchical_enabled);

    std::ostringstream os;
    os << conf;
    REQUIRE(os.str().size() > 0);
}

TEST_CASE("Test mode in radiosity USAGE", "[config]") {
    {
        const char* argv[] = {"./exec", "exact", "file"};

        std::map<std::string, docopt::value> args =
            docopt::docopt(radiosity::USAGE, {argv + 1, argv + 3});
        auto conf = RadiosityConfig::from_docopt(args);

        REQUIRE(conf.mode == RadiosityConfig::EXACT);
    }
    {
        const char* argv[] = {"./exec", "hierarchical", "file"};

        std::map<std::string, docopt::value> args =
            docopt::docopt(radiosity::USAGE, {argv + 1, argv + 3});
        auto conf = RadiosityConfig::from_docopt(args);

        REQUIRE(conf.mode == RadiosityConfig::HIERARCHICAL);
    }
}

TEST_CASE("Test mesh flag in radiosity USAGE", "[config]") {
    {
        const char* argv[] = {"./exec", "hierarchical", "--mesh=simple",
                              "file"};
        std::map<std::string, docopt::value> args =
            docopt::docopt(radiosity::USAGE, {argv + 1, argv + 4});
        auto conf = RadiosityConfig::from_docopt(args);

        REQUIRE(conf.mesh == RadiosityConfig::SIMPLE_MESH);
    }
    {
        const char* argv[] = {"./exec", "hierarchical", "--mesh=feature",
                              "file"};
        std::map<std::string, docopt::value> args =
            docopt::docopt(radiosity::USAGE, {argv + 1, argv + 4});
        auto conf = RadiosityConfig::from_docopt(args);

        REQUIRE(conf.mesh == RadiosityConfig::FEATURE_MESH);
    }
}

TEST_CASE("Test flags in radiosity USAGE", "[config]") {
    const char* argv[] = {"./exec",  "hierarchical", "-g",
                          "--links", "--exact",      "file"};

    std::map<std::string, docopt::value> args =
        docopt::docopt(radiosity::USAGE, {argv + 1, argv + 6});
    auto conf = RadiosityConfig::from_docopt(args);

    REQUIRE(conf.gouraud_enabled);
    REQUIRE(conf.links_enabled);
    REQUIRE(conf.exact_hierarchical_enabled);
}
