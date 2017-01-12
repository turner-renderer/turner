/**
 * Possible improvements:
 *
 * 1. tree.intersect can be replaced by a more performant visibility test
 * algorithm, which e.g. searches for the _first_ triangle between from and to
 * and not for the closest one, and which also tests first intersection with
 * triangles from a given candidate list (if we have found a triangle between
 * from and to, then most probably the next sample ray will also hit it).
 *
 * 2. Sampling can be done for one from triangle over a hemisphere. Every
 * triangle t hit by a sample ray, contributes to the form factor F_(from,t). In
 * that way, we don't have to make a visibility test, and we compute
 * simultaneously the row/column F_(from,_).
 *
 * 3. Use Poisson disk sampling.
 */

#include "radiosity.h"
#include "sampling.h"

float form_factor(const KDTree& tree, const Vec& from_pos, const Vec& from_u,
                  const Vec& from_v, const Vec& from_normal, const Vec& to_pos,
                  const Vec& to_u, const Vec& to_v, const Vec& to_normal,
                  const float to_area, const KDTree::TriangleId to_id,
                  const size_t num_samples) {
    float result = 0;
    for (size_t i = 0; i < num_samples; ++i) {
        auto p1 = sampling::triangle(from_pos, from_u, from_v);
        auto p2 = sampling::triangle(to_pos, to_u, to_v);

        Vec v = p2 - p1;
        if (tree.intersect(Ray{p1 + EPS * from_normal, v}) != to_id) {
            continue;
        }

        float square_length = v.SquareLength();
        if (square_length == 0) {
            continue;
        }

        float length = sqrt(square_length);

        float cos_theta1 = v * from_normal / length;
        if (cos_theta1 <= 0) {
            continue;
        }

        float cos_theta2 = (-v) * to_normal / length;
        if (cos_theta2 <= 0) {
            continue;
        }

        float G = cos_theta1 * cos_theta2 /
                  (M_PI * square_length + to_area / num_samples);
        result += G;
    }

    return result * to_area / num_samples;
}

float form_factor(const KDTree& tree, const Triangle& from, const Triangle& to,
                  const KDTree::TriangleId to_id, const size_t num_samples) {
    return form_factor(tree, from.vertices[0], from.u, from.v, from.normal,
                       to.vertices[0], to.u, to.v, to.normal, to.area(), to_id,
                       num_samples);
}

float form_factor(const KDTree& tree, const KDTree::TriangleId from_id,
                  const KDTree::TriangleId to_id, const size_t num_samples) {
    assert(from_id != to_id);
    const auto& from = tree[from_id];
    const auto& to = tree[to_id];

    return form_factor(tree, from, to, to_id, num_samples);
}

/**
 * @note A summand is missing in the formula on [CW93], 4.6, p. 73, .
 */
float form_factor_of_parallel_rects(float a, float b, float c) {
    float X = a / c;
    float Y = b / c;
    return 2.f / (M_PI * X * Y) *
           (log(sqrt((1 + X * X) * (1 + Y * Y) / (1 + X * X + Y * Y))) +
            X * sqrt(1 + Y * Y) * atan(X / sqrt(1 + Y * Y)) +
            Y * sqrt(1 + X * X) * atan(Y / sqrt(1 + X * X)) - X * atan(X) -
            Y * atan(Y));
}

float form_factor_of_orthogonal_rects(float a, float b, float c) {
    float H = a / c;
    float W = b / c;
    return 1.f / (M_PI * W) *
           (W * atan(1 / W) + H * atan(1 / H) -
            sqrt(H * H + W * W) * atan(1 / sqrt(H * H + W * W)) +
            1.f / 4 * log((1 + W * W) * (1 + H * H) / (1 + W * W + H * H) *
                          pow(W * W * (1 + W * W + H * H) /
                                  ((1 + W * W) * (W * W + H * H)),
                              W * W) *
                          pow(H * H * (1 + W * W + H * H) /
                                  ((1 + H * H) * (H * H + W * W)),
                              H * H)));
}
