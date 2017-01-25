#include "raytracer.h"
#include "lib/lambertian.h"
#include "lib/stats.h"
#include "trace.h"

Color trace(const Vec& origin, const Vec& dir, const KDTree& triangles_tree,
            const std::vector<Light>& lights, int depth,
            const TracerConfig& conf) {
    Stats::instance().num_rays += 1;

    if (depth > conf.max_recursion_depth) {
        return {};
    }

    auto& light = lights.front();

    // intersection
    float dist_to_triangle, s, t;
    auto triangle_id =
        triangles_tree.intersect(aiRay(origin, dir), dist_to_triangle, s, t);
    if (!triangle_id) {
        return conf.bg_color;
    }

    // light direction
    auto p = origin + dist_to_triangle * dir;
    auto light_dir = (light.position - p).Normalize();

    // interpolate normal
    const auto& triangle = triangles_tree[triangle_id];
    auto normal = triangle.interpolate_normal(1.f - s - t, s, t);

    // direct light
    auto direct_lightning =
        lambertian(light_dir, normal, triangle.diffuse, light.color);

    // move slightly in direction of normal
    auto p2 = p + normal * 0.0001f;

    auto color = direct_lightning;

    // reflection
    if (triangle.reflectivity > 0) {
        // compute reflected ray from incident ray
        auto reflected_ray_dir = dir - 2.f * (normal * dir) * normal;
        auto reflected_color = trace(p2, reflected_ray_dir, triangles_tree,
                                     lights, depth + 1, conf);

        color = (1.f - triangle.reflectivity) * direct_lightning +
                triangle.reflectivity * triangle.reflective * reflected_color;
    }

    // shadow
    float dist_to_next_triangle;
    light_dir = (light.position - p2).Normalize();
    float dist_to_light = (light.position - p2).Length();

    auto has_shadow = triangles_tree.intersect(aiRay(p2, light_dir),
                                               dist_to_next_triangle, s, t);

    if (has_shadow && dist_to_next_triangle < dist_to_light) {
        color -= color * conf.shadow_intensity;
    }

    return color;
}
