#include "raycaster.h"
#include "config.h"
#include "lib/stats.h"
#include "lib/triangle.h"
#include "trace.h"

Color trace(const Vec& origin, const Vec& dir,
            KDTreeIntersection& tree_intersection,
            const std::vector<Light>& /* lights */, int /* depth */,
            const TracerConfig& conf) {
    // intersection
    float dist_to_triangle, s, t;
    auto triangle_id =
        tree_intersection.intersect({origin, dir}, dist_to_triangle, s, t);
    if (!triangle_id) {
        return conf.bg_color;
    }

    Stats::instance().num_rays += 1;
    auto res = tree_intersection[triangle_id].diffuse;

    // The light is at camera position. The farther away an object the darker it
    // is. It's not visible beyond max visibility.
    res.a = clamp(1.f - (dist_to_triangle / conf.max_visibility), 0.f, 1.f);
    return res;
}
