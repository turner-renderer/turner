#include "raycaster.h"
#include "config.h"
#include "lib/intersection.h"
#include "lib/lambertian.h"
#include "lib/raster.h"
#include "lib/stats.h"
#include "lib/triangle.h"
#include "trace.h"

#include <assimp/Importer.hpp>  // C++ importer interface
#include <assimp/postprocess.h> // Post processing flags
#include <assimp/scene.h>       // Output data structure

#include <iostream>
#include <math.h>
#include <vector>

Color trace(const Vec& origin, const Vec& dir, const KDTree& triangles,
            const std::vector<Light>& /* lights */, int /* depth */,
            const TracerConfig& conf) {
    // intersection
    float dist_to_triangle, s, t;
    auto triangle_id =
        triangles.intersect(aiRay{origin, dir}, dist_to_triangle, s, t);
    if (!triangle_id) {
        return conf.bg_color;
    }

    Stats::instance().num_rays += 1;
    auto res = triangles[triangle_id].diffuse;

    // The light is at camera position. The farther away an object the darker it
    // is. It's not visible beyond max visibility.
    res.a = clamp(1.f - (dist_to_triangle / conf.max_visibility), 0.f, 1.f);
    return res;
}
