#include "trace.h"
#include "lib/lambertian.h"
#include "lib/monte_carlo.h"
#include "lib/stats.h"

Color trace(const Vec origin, const Vec dir,
        const Tree& triangles, const Vec light_pos,
        const Color light_color, int depth, const Configuration& conf)
{
    auto result = Color(0, 0, 0, 1);
    const float diffuse_portion = 0.1f;
    const float direct_portion = 1.f - diffuse_portion;

    if (depth > conf.max_depth) {
        return result;
    }

    // intersection
    float dist_to_triangle, s, t;
    auto triangle_pt = triangles.intersect(
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
    // calculate shadow
    float dist_to_next_triangle;
    auto p2 = p + normal * 0.0001f;
    light_dir = (light_pos - p2).Normalize();
    float dist_to_light = (light_pos - p2).Length();
    auto has_shadow = triangles.intersect(
        aiRay(p2, light_dir), dist_to_next_triangle, s, t);

    // Do we get direct light?
    if (!has_shadow || dist_to_next_triangle > dist_to_light) {
        result += direct_portion * lambertian(light_dir, normal, triangle.diffuse, light_color);
    }

    // ambient light
    Hemisphere hemisphere;
    const int num_samples = 8;
    auto diffuse = Color{0, 0, 0, 1};
    // Turn hemisphere according normal, i.e. Up(0, 1, 0) is turned so that
    // it lies on normal of the hit point.
    aiMatrix3x3 mTrafo;
    aiMatrix3x3::FromToMatrix(Vec(0, 1.f, 0), normal, mTrafo);
    for(int run = 0; run < num_samples; run++) {
        const Vec dir = mTrafo * hemisphere.sample();

        const auto indirect_light = trace(p2, dir, triangles, light_pos, light_color, depth + 1, conf);

        diffuse += lambertian(dir, normal, triangle.diffuse, indirect_light);
    }

    // Average AND adjust by PDF
    diffuse /= num_samples * (1.f / (2.f * M_PI));

    result += diffuse_portion * diffuse;

    return result;
}

