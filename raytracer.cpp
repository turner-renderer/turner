#include "trace.h"
#include "lib/lambertian.h"
#include "lib/stats.h"

Color trace(const Vec& origin, const Vec& dir,
        const Tree& triangles_tree, const Vec& light_pos,
        const Color& light_color, int depth, const Configuration& conf)
{
    Stats::instance().num_rays += 1;

    auto result = aiColor4D(0, 0, 0, 1);

    if (depth > conf.max_depth) {
        return result;
    }

    // intersection
    float dist_to_triangle, s, t;
    auto triangle_pt = triangles_tree.intersect(
        aiRay(origin, dir), dist_to_triangle, s, t);
    if (!triangle_pt) {
        return result;
    }

    // light direction
    auto p = origin + dist_to_triangle * dir;
    auto light_dir = (light_pos - p).Normalize();

    // interpolate normal
    const auto& triangle = *triangle_pt;
    auto normal = triangle.interpolate_normal(1.f - s - t, s, t);

    // direct light
    result += 0.9f * lambertian(
        light_dir, normal, triangle.diffuse, light_color);

    // move slightly in direction of normal
    auto p2 = p + normal * 0.0001f;

    // reflection
    // compute reflected ray from incident ray
    auto reflected_ray_dir = dir - 2.f * (normal * dir) * normal;
    auto reflected_color = triangle.diffuse * 0.1f * trace(
        p2, reflected_ray_dir,
        triangles_tree, light_pos, light_color, depth + 1, conf);
    result += reflected_color;

    // shadow
    float dist_to_next_triangle;
    light_dir = (light_pos - p2).Normalize();
    float dist_to_light = (light_pos - p2).Length();

    auto has_shadow = triangles_tree.intersect(
        aiRay(p2, light_dir), dist_to_next_triangle, s, t);

    if (has_shadow && dist_to_next_triangle < dist_to_light) {
        result -= result * conf.shadow_intensity;
    }

    return result;
}
