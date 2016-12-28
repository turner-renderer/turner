#pragma once

#include "kdtree.h"

/**
 * Numerical integration of form factor from infinitesimal area to finite area.
 *
 * Cf. [CW93], 4.9, p. 91. and Algorithm 4.21.
 *
 * The form factor between faces `from` (i) and `to` (j) is defined as
 *
 * F_ij = 1/A_i ∫_x ∫_y G'(x, y) dA_y dA_x
 *
 * Here, x and y are points on face i and j, A_i is an infinitesimal area on i
 * around x, and A_j is the area of j. Further,
 *
 * G'(x, y) = V(x, y) * cos+(θ_i) * cos+(θ_j) / (π * ||x-y||^2), where
 *
 * V(x, y) - visibility indicator between x and y (1 if visible, 0 else)
 * θ_i - angle between normal of x and vector to y (ω)
 * θ_j - angle between normal of y and vector to x (-ω)
 *
 * @note When the distance from i to j is small relative to the size of j, the
 *       result is inexact.
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
float form_factor(const KDTree& tree, const Vec& from_pos, const Vec& from_u,
                  const Vec& from_v, const Vec& from_normal, const Vec& to_pos,
                  const Vec& to_u, const Vec& to_v, const Vec& to_normal,
                  const float to_area, const KDTree::TriangleId to_id,
                  const size_t num_samples = 128);

/*
 * Same as above, with explicitly defined triangles.
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
