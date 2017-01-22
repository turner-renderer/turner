#pragma once

#include "lib/types.h"

#include <docopt/docopt.h>

#include <map>
#include <sstream>

/**
 * Common configuration
 */
struct Config {
    // image
    float aspect = 1;
    size_t width = 640;

    // common options
    size_t num_threads = 1;
    float inverse_gamma = 0.454545;
    float exposure = 1;
    Color bg_color;
    bool gamma_correction_enabled = true;

    // scene
    std::string filename;

    void check() const {
        assert(0 < aspect);
        assert(1 <= num_threads);
        assert(0 <= exposure);
    }

    /**
     * Parse color from string.
     *
     * @param  color_str contains either
     *         1. one numerical value in [0, 255], or
     *         2. three space-separated numerical values in [0, 255].
     * @return if color_str is of type 1, then return Color with constant value
     *         for all three channels, otherwise Color with channels containing
     *         corresponding parsed values.
     */
    static Color parse_color(const std::string& color_str) {
        std::vector<float> result;
        std::stringstream ss(color_str);
        std::string item;
        while (std::getline(ss, item, ' ')) {
            result.push_back(std::stof(item));
        }

        if (result.size() == 1) {
            return Color{result[0], result[0], result[0], 1};
        } else if (result.size() == 3) {
            return Color{result[0], result[1], result[2], 1};
        }
        assert(false);
        return {};
    }

    static Config
    from_docopt(const std::map<std::string, docopt::value>& args) {
        Config conf;

        conf.width = args.at("--width").asLong();
        if (args.at("--aspect")) {
            conf.aspect = std::stof(args.at("--aspect").asString());
        }

        conf.num_threads = args.at("--threads").asLong();
        conf.inverse_gamma = std::stof(args.at("--inverse-gamma").asString());
        conf.exposure = std::stof(args.at("--exposure").asString());
        conf.bg_color = parse_color(args.at("--background").asString());
        conf.gamma_correction_enabled =
            !args.at("--no-gamma-correction").asBool();

        conf.filename = args.at("<filename>").asString();

        conf.check();
        return conf;
    }
};

/**
 * Configuration of raycaster, raytracer and pathtracer.
 */
struct TracerConfig : public Config {
    TracerConfig(const Config& common_conf) : Config(common_conf) {}

    // common tracer options
    int max_recursion_depth = 3;

    // raycaster options
    float max_visibility = 2;

    // raytracer options
    float shadow_intensity = 0.5;

    // pathtracer options
    int num_pixel_samples = 1;
    int num_monte_carlo_samples = 1;

    void check() const {
        Config::check();
        assert(0 < max_recursion_depth);
        assert(0 <= max_visibility);
        assert(0 <= shadow_intensity && shadow_intensity <= 1);
        assert(1 <= num_pixel_samples);
        assert(0 <= num_monte_carlo_samples);
    }

    static TracerConfig
    from_docopt(const std::map<std::string, docopt::value>& args) {
        TracerConfig conf(Config::from_docopt(args));

        if (args.count("--max-depth")) {
            conf.max_recursion_depth = args.at("--max-depth").asLong();
        }
        if (args.count("--max-visibility")) {
            conf.max_visibility =
                std::stof(args.at("--max-visibility").asString());
        }
        if (args.count("--shadow")) {
            conf.shadow_intensity = std::stof(args.at("--shadow").asString());
        }
        if (args.count("--pixel-samples")) {
            conf.num_pixel_samples = args.at("--pixel-samples").asLong();
        }
        if (args.count("--monte-carlo-samples")) {
            conf.num_monte_carlo_samples =
                args.at("--monte-carlo-samples").asLong();
        }

        conf.check();
        return conf;
    }
};

/**
 * Radiosity configuration.
 */
struct RadiosityConfig : public Config {
    RadiosityConfig(const Config& conf) : Config(conf) {}

    enum Mode { EXACT, HIERARCHICAL } mode;

    // radiosity parameters
    float F_eps = 0.04;
    float BF_eps = 1e-6;
    size_t max_subdivisions = 3;
    size_t max_iterations = 3;
    float min_area = 1. / pow(4, max_subdivisions);

    // flags
    bool gouraud_enabled = false;
    enum Mesh { NO_MESH, SIMPLE_MESH, FEATURE_MESH } mesh = NO_MESH;
    bool features_mesh_enabled = false;
    bool links_enabled = false;
    bool exact_hierarchical_enabled = false;

    void check() const {
        Config::check();
        assert(0 < F_eps);
        assert(0 < BF_eps);
        assert(0 < min_area);
    }

    static RadiosityConfig
    from_docopt(const std::map<std::string, docopt::value>& args) {
        RadiosityConfig conf(Config::from_docopt(args));

        conf.mode = args.at("exact").asBool() ? EXACT : HIERARCHICAL;
        assert(args.at("hierarchical").asBool() == !args.at("exact").asBool());

        // radiosity parameters
        if (args.count("--form-factor-eps")) {
            conf.F_eps = std::stof(args.at("--form-factor-eps").asString());
        }
        if (args.count("--rad-shoot-eps")) {
            conf.BF_eps = std::stof(args.at("--rad-shoot-eps").asString());
        }
        if (args.count("--max-subdivisions")) {
            conf.max_subdivisions =
                std::stof(args.at("--max-subdivisions").asString());
        }
        if (args.count("--max-iterations")) {
            conf.max_iterations =
                std::stof(args.at("--max-iterations").asString());
        }

        // extra options
        if (args.count("--gouraud")) {
            conf.gouraud_enabled = args.at("--gouraud").asBool();
        }
        if (args.count("--mesh")) {
            if (!args.at("--mesh")) {
                conf.mesh = NO_MESH;
            } else if (args.at("--mesh").asString() == "simple") {
                conf.mesh = SIMPLE_MESH;
            } else if (args.at("--mesh").asString() == "feature") {
                conf.mesh = FEATURE_MESH;
            } else {
                assert(!"wrong mesh option");
            }
        }
        if (args.count("--links")) {
            conf.links_enabled = args.at("--links").asBool();
        }
        if (args.count("--exact")) {
            conf.exact_hierarchical_enabled = args.at("--exact").asBool();
        }

        conf.check();
        return conf;
    }
};
