#include <assimp/types.h>

//
// Return -1 if there is no intersection, otherwise return r s.t. the
// intersection point is P0 + r * (P1 - P0).
//
// If n is normalized, then r is exactly the euklidean distance from the ray to
// the plane.
//
// Args:
//   P0, P1: Points defining a ray from P0 to P1
//   V0, n: Plane through V0 with normal n
//
// Cf. http://geomalgorithms.com/a06-_intersect-2.html
//
float ray_plane_intersection(
    const aiVector3D& P0, const aiVector3D& P1,
    const aiVector3D& V0, const aiVector3D& n)
{
    auto denom = n * (P1 - P0);
    if (denom == 0) {
        return -1;
    }

    auto nom = n * (V0 - P0);
    auto r = nom / denom;

    return r < 0 ? -1 : r;
}


//
// Return -1 if there is no intersection, otherwise return r s.t. the
// intersection point is P0 + r * (P1 - P0).
//
// If n is normalized, then r is exactly the euklidean distance from the ray to
// the triangle.
//
// Args:
//   P0, P1: Points defining a ray from P0 to P1
//   V0, V1, V2: Points defining a triangle
//
// Cf. http://geomalgorithms.com/a06-_intersect-2.html
//
float ray_triangle_intersection(
    const aiVector3D& P0, const aiVector3D& P1,
    const aiVector3D& V0, const aiVector3D& V1, const aiVector3D& V2)
{
    auto u = V1 - V0;
    auto v = V2 - V0;
    auto n = u^v;  // normal vector of the triangle

    auto r = ray_plane_intersection(P0, P1, V0, n);
    if (r == -1) {
        return -1;
    }

    auto P_int = P0 + r * (P1 - P0);
    auto w = P_int - V0;

    // precompute scalar products
    auto uv = u*v;
    auto wv = w*v;
    auto vv = v*v;
    auto wu = w*u;
    auto uu = u*u;

    auto denom = uv * uv - uu * vv;
    auto s = (uv * wv - vv * wu) / denom;

    if (s < 0) {
        return -1;
    }

    auto t = (uv * wu - uu * wv) / denom;

    if (t < 0 || s + t > 1) {
        return -1;
    }

    return r;
}