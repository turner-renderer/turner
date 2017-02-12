#pragma once

#include "config.h"
#include "lib/kdtree.h"
#include "lib/types.h"

/**
 * Main function of a tracer.
 *
 * Used in the main routine for tracing (cf. main.cpp).
 *
 * TODO: Could be a performance bottleneck since not inlined. Profile!
 *
 * @param  origin            origin of the ray
 * @param  dir               direction of the ray
 * @param  tree_intersection wrapped kd-tree containing triangles for
 *                           intersection computations
 * @param  lights            all lights in the scene
 * @param  depth             recursion depth
 * @param  conf              configuration
 * @return                   Color hit by the ray
 */
Color trace(const Vec& origin, const Vec& dir,
            KDTreeIntersection& tree_intersection,
            const std::vector<Light>& lights, int depth,
            const TracerConfig& conf);
