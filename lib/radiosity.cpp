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

    float sum = 0;
    for (size_t i = 0; i != num_samples; ++i) {
        auto p1 = sampling::triangle(from);
        auto p2 = sampling::triangle(to);

        auto v = p2 - p1;
        float r, unused;
        auto intersect_id =
            tree.intersect(Ray{p1 + v/2.f, v}, r, unused, unused);
        if (!(intersect_id == to_id)) {
            continue;
        }

        auto square_length = v.SquareLength();

        v.Normalize();
        float cos_theta1 = v * from.normal;
        float cos_theta2 = -v * to.normal;
        // float delta_F = cos_theta1 * cos_theta2 / (
        // M_PI * square_length + to.area() / num_samples);
        // if (delta_F > 0) {
        // sum += delta_F;
        // }

        float G = cos_theta1 * cos_theta2 / square_length;
        if (G > 0) {
            sum += G;
        }
    }

    // return sum * from.area() / num_samples;
    return M_1_PI * sum * to.area() / num_samples;
}
