#include "trace.h"
#include "lib/lambertian.h"
#include "lib/stats.h"
#include "lib/xorshift.h"

Color trace(const Vec& origin, const Vec& dir,
        const Tree& triangles_tree, const std::vector<Light>& lights,
        int depth, const Configuration& conf)
{
    //xorshift64star<float> uniform{4};

    Stats::instance().num_rays += 1;

    if (depth > conf.max_depth) {
        return {};
    }

    // intersection
    float dist_to_triangle, s, t;
    auto triangle_pt = triangles_tree.intersect(
        aiRay(origin, dir), dist_to_triangle, s, t);
    if (!triangle_pt) {
        return conf.bg_color;
    }

    // draw edge of mesh
    // if one barycentric coordinate is close to 0.0 we are close to a border
    float r = 1.f - s - t;
    float EPS = 0.005f;
    if ( r < EPS || s < EPS || t < EPS) {
        return {};
    }

    // simple raycast
    const auto& triangle = *triangle_pt;
    auto result = triangle.diffuse;
    result.a = dist_to_triangle;
    //Color result { uniform(), uniform(), uniform(), dist_to_triangle};
    return result;
}
