#include "raytracer.h"
#include "lib/lambertian.h"
#include "lib/stats.h"
#include "trace.h"

Color trace(const Point3f& origin, const Vector3f& dir,
            KDTreeIntersection& tree_intersection,
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
        tree_intersection.intersect({origin, dir}, dist_to_triangle, s, t);
    if (!triangle_id) {
        return conf.bg_color;
    }

    // light direction
    auto p = origin + dist_to_triangle * dir;
    Vector3f light_dir = normalize(light.position - p);

    // interpolate normal
    const auto& triangle = tree_intersection[triangle_id];
    Normal3f normal = triangle.interpolate_normal(1.f - s - t, s, t);

    // direct light
    auto direct_lightning =
        lambertian(light_dir, normal, triangle.diffuse, light.color);

    // move slightly in direction of normal
    auto p2 = p + Vector3f(normal) * 0.0001f;

    auto color = direct_lightning;

    // reflection
    if (triangle.reflectivity > 0) {
        // compute reflected ray from incident ray
        Vector3f reflected_ray_dir =
            dir - Vector3f(2.f * dot(normal, dir) * normal);
        auto reflected_color = trace(p2, reflected_ray_dir, tree_intersection,
                                     lights, depth + 1, conf);

        color = (1.f - triangle.reflectivity) * direct_lightning +
                triangle.reflectivity * triangle.reflective * reflected_color;
    }

    // shadow
    float dist_to_next_triangle;
    light_dir = normalize(light.position - p2);
    float dist_to_light = (light.position - p2).length();

    auto has_shadow = tree_intersection.intersect({p2, light_dir},
                                                  dist_to_next_triangle, s, t);

    if (has_shadow && dist_to_next_triangle < dist_to_light) {
        color -= color * conf.shadow_intensity;
    }

    return color;
}
