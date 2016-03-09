#pragma once

#include "lib/kdtree.h"
#include "lib/types.h"
#include <vector>
#include <sstream>

// using Tree = KDTree<10>;
using Tree = FastKDTree;

class Configuration {

public:

    Configuration(
            long max_depth,
            float shadow_intensity,
            long num_pixel_samples,
            long num_monte_carlo_samples,
            long num_threads,
            const std::string& bg_color_str,
            float inverse_gamma,
            const bool no_gamma_correction):
        max_depth(max_depth),
        shadow_intensity(shadow_intensity),
        num_pixel_samples(num_pixel_samples),
        num_monte_carlo_samples(num_monte_carlo_samples),
        num_threads(num_threads),
        inverse_gamma(inverse_gamma),
        gamma_correction_enabled(!no_gamma_correction)
    {
        check();

        // Parse background color.
        std::vector<float> result;
        std::stringstream ss(bg_color_str);
        std::string item;
        while (std::getline(ss, item, ' ')) {
            result.push_back(std::stof(item));
        }

        if (result.size() == 1) {
            bg_color = Color{result[0], result[0], result[0], 1};
        } else if (result.size() == 3) {
            bg_color = Color{result[0], result[1], result[2], 1};
        } else {
            assert(false);
        }
    }

private:

    void check() {
        assert(0 < max_depth);
        assert(0 <= shadow_intensity && shadow_intensity <= 1);
        assert(1 <= num_pixel_samples);
        assert(0 <= num_monte_carlo_samples);
        assert(1 <= num_threads);
    }

public:

    int max_depth;
    float shadow_intensity;
    int num_pixel_samples;
    int num_monte_carlo_samples;
    int num_threads;
    float inverse_gamma;
    bool gamma_correction_enabled;
    Color bg_color;
};

Color trace(const Vec& origin, const Vec& dir,
        const Tree& triangles_tree, const std::vector<Light>& lights,
        int depth, const Configuration& conf);

