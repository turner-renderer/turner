#include "types.h"


float eps_zero(float a) {
    return std::abs(a) < EPS;
}

Vec operator/(int a, const Vec& v) {
    return {
        static_cast<float>(a)/v.x,
        static_cast<float>(a)/v.y,
        static_cast<float>(a)/v.z
    };
}

float fmin(float x, float y, float z) {
    return std::fmin(x, std::min(y, z));
}

float fmax(float x, float y, float z) {
    return std::fmax(x, std::max(y, z));
}

Box operator+(const Box& a, const Box& b) {
    Vec new_min =
        { std::min(a.min.x, b.min.x)
        , std::min(a.min.y, b.min.y)
        , std::min(a.min.z, b.min.z)
        };

    Vec new_max =
        { std::max(a.max.x, b.max.x)
        , std::max(a.max.y, b.max.y)
        , std::max(a.max.z, b.max.z)
        };

    return {new_min, new_max};
}


// Ray with precomputed inverse direction
Ray::Ray(const Vec& pos, const Vec& dir)
    : aiRay(pos, dir)
    , invdir(1/dir)
    {}

Ray::Ray(const aiRay& ray) : Ray(ray.pos, ray.dir) {}

//
// Convert 2d raster coodinates into 3d cameras coordinates.
//
// We assume that the camera is trivial:
//   assert(cam.mPosition == (0, 0, 0))
//   assert(cam.mUp == (0, 1, 0))
//   assert(cam.mLookAt == (0, 0, -1))
//   assert(cam.mAspect != 0)
//
// The positioning of the camera is done in its parent's node
// transformation matrix.
//
aiVector3D Camera::raster2cam(const aiVector2D& p, const float w, const float h) const
{
    return trafo_ * aiVector3D(
        -delta_x_ * (1 - 2 * p.x / w),
        delta_y_ * (1 - 2 * p.y / h),
        -1);
}
