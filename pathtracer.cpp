#include "pathtracer.h"
#include "lib/lambertian.h"
#include "lib/sampling.h"
#include "lib/stats.h"
#include "trace.h"

/**
 * Return color of the object hit by (origin, dir) ray.
 *
 * The color is calculated using the Monte-Carlo approximation of the light
 * equation, thefore it is not guaranteed that the calculated color values are
 * less than 1. E.g. an approximation of value 1 may be greater than 1.
 */
Color trace(const Vec& origin, const Vec& dir, const KDTree& triangles,
            const std::vector<Light>& lights, int depth,
            const TracerConfig& conf) {
    if (depth > conf.max_recursion_depth) {
        return {};
    }

    Stats::instance().num_rays += 1;

    // intersection
    float dist_to_triangle, s, t;
    auto triangle_id =
        triangles.intersect(Ray(origin, dir), dist_to_triangle, s, t);
    if (!triangle_id) {
        return conf.bg_color;
    }

    auto p = origin + dist_to_triangle * dir;

    // interpolate normal
    const auto& triangle = triangles[triangle_id];
    auto normal = triangle.interpolate_normal(1.f - s - t, s, t);

    auto p2 = p + normal * 0.0001f;

    //
    // Direct lightning
    //

    Color direct_lightning;

    for (auto& light : lights) {
        // light direction
        auto light_dir = (light.position - p).Normalize();
        float dist_to_light = (light.position - p2).Length();
        float dist_to_next_triangle;
        auto has_shadow = triangles.intersect(aiRay(p2, light_dir),
                                              dist_to_next_triangle, s, t);

        // Do we get direct light?
        if (!has_shadow || dist_to_next_triangle > dist_to_light) {
            // lambertian
            direct_lightning = std::max(0.f, light_dir * normal) * light.color;
        }
    }

    //
    // Indirect lightning (via Monte Carlo sampling)
    //

    Color indirect_lightning;

    // Turn hemisphere according normal, i.e. Up(0, 0, 1) is turned so that
    // it lies on normal of the hit point.
    aiMatrix3x3 mTrafo;
    aiMatrix3x3::FromToMatrix(Vec{0, 0, 1}, normal, mTrafo);

    for (int run = 0; run < conf.num_monte_carlo_samples; run++) {
        auto dir_theta = sampling::hemisphere();
        auto dir = mTrafo * dir_theta.first;
        auto cos_theta = dir_theta.second;

        const auto indirect_light =
            trace(p2, dir, triangles, lights, depth + 1, conf);

        // lambertian
        indirect_lightning += cos_theta * indirect_light;
    }
    // We don't divide by CDF 1/2π here, since it is better to do it in the
    // next expression.
    indirect_lightning /= static_cast<float>(conf.num_monte_carlo_samples);

    //
    // Light equation
    //
    // ∫ L(p,ω) ρ/π dω
    //   ≈ ρ/π (L_direct(p,ω_light) + 1/N ∑ L(p,ω_sample)/(1/2π))
    //   = ρ (L_direct(p,ω_light)/π + 2/N ∑ L(p,ω_sample))
    //
    // N - number of samples
    // ρ - material color
    //
    return triangle.diffuse * (direct_lightning * static_cast<float>(M_1_PI) +
                               indirect_lightning * 2.f);
}
