#pragma once

#include "lib/kdtree.h"
#include "lib/types.h"

using Tree = KDTree<10>;

struct Configuration {
    int max_depth;
    float shadow_intensity;
    int num_pixel_samples;
    int num_monte_carlo_samples;
    int num_threads;

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

