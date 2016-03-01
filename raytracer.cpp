#include "trace.h"
#include "lib/lambertian.h"
#include "lib/stats.h"

Color trace(const Vec& origin, const Vec& dir,
        const Tree& triangles_tree, const std::vector<Light>& lights,
        int depth, const Configuration& conf)
{
    Stats::instance().num_rays += 1;

    if (depth > conf.max_depth) {
        return {};
    }

    auto& light = lights.front();

    // intersection
    float dist_to_triangle, s, t;
    auto triangle_pt = triangles_tree.intersect(
        aiRay(origin, dir), dist_to_triangle, s, t);
    if (!triangle_pt) {
        return conf.bg_color;
    }

    // light direction
    auto p = origin + dist_to_triangle * dir;
    auto light_dir = (light.position - p).Normalize();

    // interpolate normal
    const auto& triangle = *triangle_pt;
    auto normal = triangle.interpolate_normal(1.f - s - t, s, t);

    // direct light
    auto direct_lightning = 0.9f * lambertian(
        light_dir, normal, triangle.diffuse, light.color);

    // move slightly in direction of normal
    auto p2 = p + normal * 0.0001f;

    // reflection
    // compute reflected ray from incident ray
    auto reflected_ray_dir = dir - 2.f * (normal * dir) * normal;
    auto reflected_color = triangle.diffuse * 0.1f * trace(
        p2, reflected_ray_dir,
        triangles_tree, lights, depth + 1, conf);

    auto result = direct_lightning + reflected_color;

    // shadow
    float dist_to_next_triangle;
    light_dir = (light.position - p2).Normalize();
    float dist_to_light = (light.position - p2).Length();

    auto has_shadow = triangles_tree.intersect(
        aiRay(p2, light_dir), dist_to_next_triangle, s, t);

    if (has_shadow && dist_to_next_triangle < dist_to_light) {
        result -= result * conf.shadow_intensity;
    }

    return result;
}
