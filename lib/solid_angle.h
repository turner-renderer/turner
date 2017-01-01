#pragma once

#include "mesh.h"

#include <cmath>

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
float solid_angle(const Point& O, const Point& A, const Point& B,
                  const Point& C) {
    constexpr float M_2PI = 2 * M_PI;

    // Project triangle vetrices onto unit sphere around O
    const auto a = (A - O).normalize();
    const auto b = (B - O).normalize();
    const auto c = (C - O).normalize();

    float tan_omega_2 =
        std::abs(a | (b % c)) / (1 + (b | c) + (a | c) + (a | b));
    float omega = 2 * std::atan(tan_omega_2);
    return omega < 0 ? omega + M_2PI : omega;
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
float solid_angle_using_dihedral_angles(const Point& O, const Point& A,
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

    return phi_ab + phi_bc + phi_ac - M_PI;
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
float solid_angle_using_sides_angles(const Point& O, const Point& A,
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
