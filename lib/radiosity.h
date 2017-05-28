#pragma once

/**
 * Possible improvements of form factor computations:
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

#include "kdtree.h"
#include "mesh.h"
#include "sampling.h"
#include "turner.h"

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
inline float form_factor(KDTreeIntersection& tree, const Point3f& from_pos,
                         const Vector3f& from_u, const Vector3f& from_v,
                         const Normal3f& from_normal, const Point3f& to_pos,
                         const Vector3f& to_u, const Vector3f& to_v,
                         const Normal3f& to_normal, const float to_area,
                         const KDTree::TriangleId to_id,
                         const size_t num_samples = 128) {
    float result = 0;
    for (size_t i = 0; i < num_samples; ++i) {
        auto p1 = sampling::triangle(from_pos, from_u, from_v);
        auto p2 = sampling::triangle(to_pos, to_u, to_v);

        Vector3f v = p2 - p1;
        if (tree.intersect(Ray{p1 + Vector3f(EPS * from_normal), v}) != to_id) {
            continue;
        }

        float square_length = v.length_squared();
        if (square_length == 0) {
            continue;
        }

        float length = sqrt(square_length);

        float cos_theta1 = dot(v, from_normal) / length;
        if (cos_theta1 <= 0) {
            continue;
        }

        float cos_theta2 = dot(-v, to_normal) / length;
        if (cos_theta2 <= 0) {
            continue;
        }

        float G = cos_theta1 * cos_theta2 /
                  (PI * square_length + to_area / num_samples);
        result += G;
    }

    return result * to_area / num_samples;
}

/*
 * Same as above, with explicitly defined triangles.
 */
inline float form_factor(KDTreeIntersection& tree, const Triangle& from,
                         const Triangle& to, const KDTree::TriangleId to_id,
                         const size_t num_samples = 128) {
    return form_factor(tree, from.vertices[0], from.u, from.v, from.normal,
                       to.vertices[0], to.u, to.v, to.normal, to.area(), to_id,
                       num_samples);
}

/**
 * Same as above, except the triangles are contained in tree and defined by
 * corresponding ids.
 */
inline float form_factor(KDTreeIntersection& tree,
                         const KDTree::TriangleId from_id,
                         const KDTree::TriangleId to_id,
                         const size_t num_samples = 128) {
    assert(from_id != to_id);
    const auto& from = tree[from_id];
    const auto& to = tree[to_id];

    return form_factor(tree, from, to, to_id, num_samples);
}

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
inline float form_factor_of_parallel_rects(float a, float b, float c) {
    float X = a / c;
    float Y = b / c;
    return 2.f / (PI * X * Y) *
           (log(sqrt((1 + X * X) * (1 + Y * Y) / (1 + X * X + Y * Y))) +
            X * sqrt(1 + Y * Y) * atan(X / sqrt(1 + Y * Y)) +
            Y * sqrt(1 + X * X) * atan(Y / sqrt(1 + X * X)) - X * atan(X) -
            Y * atan(Y));
}

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
inline float form_factor_of_orthogonal_rects(float a, float b, float c) {
    float H = a / c;
    float W = b / c;
    return 1.f / (PI * W) *
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

using Point = RadiosityMesh::Point;

/**
 * Compute the solid angle subtended by triangle from `O`.
 *
 * See Eriksson, Folke (1990). "On the measure of solid angles". Math. Mag. 63
 * (3): 184–187.
 *
 * @param  O       origin
 * @param  A, B, C vertices of the triangle
 * @return         solid angle
 */
inline float solid_angle(const Point& O, const Point& A, const Point& B,
                         const Point& C) {
    // Project triangle vetrices onto unit sphere around O
    const auto a = (A - O).normalize();
    const auto b = (B - O).normalize();
    const auto c = (C - O).normalize();

    float tan_omega_2 =
        std::abs(a | (b % c)) / (1 + (b | c) + (a | c) + (a | b));
    float omega = 2 * std::atan(tan_omega_2);
    return omega < 0 ? omega + PI2 : omega;
}

/**
 * Compute the solid angle subtended by triangle from `O`.
 *
 * Cf. Girard's theorem and spherical excess.
 *
 * This is a reference implementation.
 *
 * @note The computation is done using dihedral angles between the planes that
 * contain the dihedral faces of the tetrahedron OABC, where A, B, C are the
 * vertices of the triangle.
 *
 * @note May be numerically instable, due to acos(x) with |x| > 1.
 *
 * @param  O       origin
 * @param  A, B, C vertices of the triangle
 * @return         solid angle
 */
inline float solid_angle_using_dihedral_angles(const Point& O, const Point& A,
                                               const Point& B, const Point& C) {
    // sides of the tetrahedron
    const auto a = A - O;
    const auto b = B - O;
    const auto c = C - O;

    // normals of the dihedral faces
    const auto n_OAC = (a % c).normalize();
    const auto n_OBC = (b % c).normalize();
    const auto n_OAB = (a % b).normalize();

    // angles between dihedral faces
    float phi_ab = std::acos(n_OAC | n_OBC);
    float phi_ac = std::acos(n_OAB | -n_OBC); // n_OCB = -n_OBC
    float phi_bc = std::acos(n_OAB | n_OAC);  // n_OBA = -n_OAB, and
                                              // n_OCA = -n_OAC

    return phi_ab + phi_bc + phi_ac - PI;
}

/**
 * Compute the solid angle subtended by triangle from `O`.
 *
 * Cf. L'Huilier's theorem and Heron's formular.
 *
 * This is a reference implementation.
 *
 * @note The computation is done using the angles between the sides OA, OB, and
 * OC, where A, B, C are the vertices of the triangle.
 *
 * @note May be numerically instable, e.g. the spherical triangle inequality may
 * be not satisfied due to numerical errors.
 *
 * @param  O       origin
 * @param  A, B, C vertices of the triangle
 * @return         solid angle
 */
inline float solid_angle_using_sides_angles(const Point& O, const Point& A,
                                            const Point& B, const Point& C) {
    // normalized sides of the tetrahedron
    const auto a = (A - O).normalize();
    const auto b = (B - O).normalize();
    const auto c = (C - O).normalize();

    // angles between the sides
    float theta_a = std::acos(b | c); // BOC
    float theta_b = std::acos(a | c); // AOC
    float theta_c = std::acos(a | b); // AOB

    // check spherical triangle inequality
    // assert(theta_a < theta_b + theta_c);
    // assert(theta_b < theta_a + theta_c);
    // assert(theta_c < theta_a + theta_b);

    float theta_s = (theta_a + theta_b + theta_c) / 2;
    return 4 * std::atan(std::sqrt(std::tan(theta_s / 2) *
                                   std::tan((theta_s - theta_a) / 2) *
                                   std::tan((theta_s - theta_b) / 2) *
                                   std::tan((theta_s - theta_c) / 2)));
}
