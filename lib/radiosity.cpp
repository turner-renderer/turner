/**
 * Work in progress:
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

/**
 * The form factor between faces `from` (i) and `to` (j) is defined as
 *
 * F_ij = 1/A_i ∫_x ∫_y G'(x, y) dA_y dA_x
 *      ≈ 1/A_j A_j/L ∑_{l = 1...L} (A_i/K ∑_{k = 1...K} G'(x^l, y^k))
 *      = A_j/N ∑_{n = 1...N} G'(x^n, y^n)
 *
 * Here, x and y are points on face i and j, A_i is the area of i, and A_i, and
 * A_j are infinitesimal areas around x and y. Further,
 *
 * G'(x, y) = V(x, y) * cos+(θ_i) * cos+(θ_j) / (π * ||x-y||^2), where
 *
 * V(x, y) - visibility indicator between x and y (1 if visible, 0 else)
 * θ_i - angle between normal of x and vector to y (ω)
 * θ_j - angle between normal of y and vector to x (-ω)
 *
 * The above integrals are approximated by Monte-Carlo.
 */
float form_factor(const KDTree& tree, const KDTree::TriangleId from_id,
                  const KDTree::TriangleId to_id, const size_t num_samples) {
    assert(from_id != to_id);

    const auto& from = tree[from_id];
    const auto& to = tree[to_id];

    float result = 0;
    for (size_t i = 0; i != num_samples; ++i) {
        auto p1 = sampling::triangle(from);
        auto p2 = sampling::triangle(to);

        auto v = p2 - p1;
        if (tree.intersect(Ray{p1 + EPS * v, v}) != to_id) {
            continue;
        }

        auto square_length = v.SquareLength();

        v.Normalize();
        float cos_theta1 = v * from.normal;
        float cos_theta2 = -v * to.normal;

        float G = cos_theta1 * cos_theta2 / square_length;
        if (G > 0) {
            result += G;
        }
    }

    return M_1_PI * result * to.area() / num_samples;
}

/**
 * As above function.
 *
 * Instead of using triangles from the given kd-tree, computes the form factors
 * for the directly given triangles. The triangle `to` has to be a subtriangle
 * of the triangle in the kd-tree defined by `to_id`.
 */
float form_factor(const KDTree& tree, const Triangle& from, const Triangle& to,
                  const KDTree::TriangleId to_id, const size_t num_samples) {
    float result = 0;
    for (size_t i = 0; i != num_samples; ++i) {
        auto p1 = sampling::triangle(from);
        auto p2 = sampling::triangle(to);

        auto v = p2 - p1;
        if (tree.intersect(Ray{p1 + EPS * v, v}) != to_id) {
            continue;
        }

        auto square_length = v.SquareLength();

        v.Normalize();
        float cos_theta1 = v * from.normal;
        float cos_theta2 = -v * to.normal;

        float G = cos_theta1 * cos_theta2 / square_length;
        if (G > 0) {
            result += G;
        }
    }

    return M_1_PI * result * to.area() / num_samples;
}

/**
 * [CW93], Algorithm 4.21 without obvious mistakes.
 */
float form_factor_expiremental(const KDTree& tree,
                               const KDTree::TriangleId from_id,
                               const KDTree::TriangleId to_id,
                               const size_t num_samples) {
    assert(from_id != to_id);

    const auto& from = tree[from_id];
    const auto& to = tree[to_id];

    float result = 0;
    for (size_t i = 0; i != num_samples; ++i) {
        auto p1 = sampling::triangle(from);
        auto p2 = sampling::triangle(to);

        auto v = p2 - p1;
        if (tree.intersect(Ray{p1 + EPS * v, v}) != to_id) {
            continue;
        }

        auto square_length = v.SquareLength();

        v.Normalize();
        float cos_theta1 = v * from.normal;
        float cos_theta2 = -v * to.normal;
        float delta_F = cos_theta1 * cos_theta2 /
                        (M_PI * square_length + to.area() / num_samples);
        if (delta_F > 0) {
            result += delta_F;
        }
    }

    return result * from.area() / num_samples;
}
