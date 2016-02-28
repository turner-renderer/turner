#pragma once

#include "lib/kdtree.h"
#include "lib/types.h"
#include <vector>
#include <sstream>

using Tree = KDTree<10>;

struct Configuration {
    int max_depth;
    float shadow_intensity;
    int num_pixel_samples;
    int num_monte_carlo_samples;
    int num_threads;
    Color bg_color;

    void parse_bg_color(const std::string& str) {
        std::vector<float> result;
        std::stringstream ss(str);
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

    void check() {
        assert(0 < max_depth);
        assert(0 <= shadow_intensity && shadow_intensity <= 1);
        assert(1 <= num_pixel_samples);
        assert(0 <= num_monte_carlo_samples);
        assert(1 <= num_threads);
    }
};

Color trace(const Vec& origin, const Vec& dir,
        const Tree& triangles_tree, const Vec& light_pos,
        const Color& light_color, int depth, const Configuration& conf);

