#pragma once

#include "kdtree.h"

/**
 * The form factor between faces `from` (i) and `to` (j) is defined as
 *
 * F_ij = 1/A_i ∫_x ∫_y G'(x, y) dA_y dA_x
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
 * @param  tree        Kd-tree used for determining V(x, y)
 * @param  from        Triangle i (does not need to be contained in tree)
 * @param  to          Triangle j (does not need to be container in tree, cf.
 *                     to_id)
 * @param  to_id       id of a triangle contained in tree, s.t. `to` is its
 *                     subtriangle
 * @param  num_samples Number of samples in Monte Carlo approximation, i.e.
 *                     number random tuples (x, y) s.t. x is on the triangle
 *                     `from` and y is on the triangle `to`.
 * @return             form factor F_ij
 */
float form_factor(const KDTree& tree, const Triangle& from, const Triangle& to,
                  const KDTree::TriangleId to_id,
                  const size_t num_samples = 128);

/**
 * Same as above, except the triangles are contained in tree and defined by
 * corresponding ids.
 */
float form_factor(const KDTree& tree, const KDTree::TriangleId from_id,
                  const KDTree::TriangleId to_id,
                  const size_t num_samples = 128);

/**
 * Compute form factor of two parallel rectangles with sides a, b analytically.
 *
 * Cf. [CW93], 4.6, p. 73
 *
 * @param  a side of rectangles
 * @param  b side of rectangles
 * @param  c distance of rectangles
 * @return   form factor
 */
float form_factor_of_parallel_rects(float a, float b, float c);

/**
 * Compute form factor of orthogonal rectangles with sides a, c and b, c
 * analytically.
 *
 * Note: Both rectangles have the same side c, and a and b are orthogonal.
 *
 * Cf. [CW93], 4.6, p. 73
 *
 * @param  a side of rectangles
 * @param  b side of rectangles
 * @param  c distance of rectangles
 * @return   form factor
 */
float form_factor_of_orthogonal_rects(float a, float b, float c);
