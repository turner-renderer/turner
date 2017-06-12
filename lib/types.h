#pragma once

#include "../src/geometry.h"

#include <assimp/camera.h>
#include <assimp/types.h>

#include <array>
#include <assert.h>
#include <vector>

#define UNUSED(x) (void)(x)

static constexpr float EPS = 0.00001f;
static constexpr float FLT_MAX = std::numeric_limits<float>::max();

inline bool is_eps_zero(float a) { return std::abs(a) < EPS; }

/**
 * If a is almost zero, return 0.
 */
inline float eps_zero(float a) {
    if (is_eps_zero(a)) {
        return 0;
    }
    return a;
}

template <typename Number>
Number clamp(Number input, Number min = 0, Number max = 255) {
    if (input < min) {
        return min;
    } else if (max < input) {
        return max;
    }
    return input;
}

inline constexpr float min(float a, float b, float c) {
    return a < b ? (a < c ? a : c) : (b < c ? b : c);
}

inline constexpr float max(float a, float b, float c) {
    return a > b ? (a > c ? a : c) : (b > c ? b : c);
}

static constexpr auto PI = turner::PI;

using Vec = turner::Vector3f;
using Axis = turner::Axis3;
static constexpr auto AXES = turner::AXES3;

namespace cereal {
template <class Archive> void serialize(Archive& archive, Vec& v) {
    archive(v.x, v.y, v.z);
}
} // namespace sereal

using Color = aiColor4D;

inline Color operator/(const Color& c, size_t x) {
    return c / static_cast<float>(x);
}

inline float fmin(float x, float y, float z) {
    return std::fmin(x, std::min(y, z));
}

inline float fmax(float x, float y, float z) {
    return std::fmax(x, std::max(y, z));
}

template <class Archive> void serialize(Archive& archive, Color& c) {
    archive(c.r, c.g, c.b, c.a);
}

/**
 * Axes aligned bounding box (AABB).
 */
struct Box {
    float surface_area() const {
        auto e0 = max[0] - min[0];
        auto e1 = max[1] - min[1];
        auto e2 = max[2] - min[2];

        if (is_eps_zero(e0)) {
            return e1 * e2;
        } else if (is_eps_zero(e1)) {
            return e0 * e2;
        } else if (is_eps_zero(e2)) {
            return e0 * e1;
        }

        return 2.f * (e0 * e1 + e0 * e2 + e1 * e2);
    }

    /**
     * Check if the box is planar in the plane: ax = 0.
     */
    bool is_planar(Axis ax) const {
        return !(std::abs((max - min)[static_cast<int>(ax)]) > EPS);
    }

    bool is_trivial() const {
        return is_eps_zero(min[0]) && is_eps_zero(min[1]) &&
               is_eps_zero(min[2]) && is_eps_zero(max[0]) &&
               is_eps_zero(max[1]) && is_eps_zero(max[2]);
    }

    /**
     * Split `box` on the plane defined by: plane_ax = plane_pos.
     */
    std::pair<Box, Box> split(Axis plane_ax, float plane_pos) const {
        size_t ax = static_cast<size_t>(plane_ax);
        assert(this->min[ax] - EPS <= plane_pos);
        assert(plane_pos <= this->max[ax] + EPS);

        Vec lmax = this->max;
        lmax[ax] = plane_pos;
        Vec rmin = this->min;
        rmin[ax] = plane_pos;

        return {{this->min, lmax}, {rmin, this->max}};
    }

    template <class Archive> void serialize(Archive& archive) {
        archive(min, max);
    }

    Vec min, max;
};

/**
 * Compute the minimal AABB containing both `a` and `b`.
 */
inline Box operator+(const Box& a, const Box& b) {
    Vec new_min = {std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y),
                   std::min(a.min.z, b.min.z)};

    Vec new_max = {std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y),
                   std::max(a.max.z, b.max.z)};

    return {new_min, new_max};
}

/**
 * Light Source
 */
struct Light {
    Vec position;
    Color color;
};

/**
 * Ray with precomputed inverse direction
 */
struct Ray {
    Ray(const Vec& pos, const Vec& dir)
        : pos(pos), dir(dir), invdir(1 / dir.x, 1 / dir.y, 1 / dir.z) {}

    Vec pos, dir, invdir;
};

/**
 * Represents a camera including transformation
 */
class Camera : public aiCamera {
public:
    template <typename... Args>
    Camera(const aiMatrix4x4& trafo, Args... args)
        : aiCamera{args...}
        , trafo_(trafo) // discard translation in trafo
        , inverse_trafo_(trafo_) {
        assert(mPosition == aiVector3D());
        assert(mUp == aiVector3D(0, 1, 0));
        assert(mLookAt == aiVector3D(0, 0, -1));
        assert(mAspect != 0);

        mPosition = trafo * mPosition;
        inverse_trafo_.Inverse();
        delta_x_ = tan(mHorizontalFOV);
        delta_y_ = delta_x_ / mAspect;
    }

    /**
     * Convert 2d raster coodinates into 3d world coordinates.
     *
     * We assume that the camera is trivial:
     *   assert(cam.mPosition == (0, 0, 0))
     *   assert(cam.mUp == (0, 1, 0))
     *   assert(cam.mLookAt == (0, 0, -1))
     *   assert(cam.mAspect != 0)
     *
     * The positioning of the camera is done in its parent's node
     * transformation matrix.
     */
    Vec raster2cam(const aiVector2D& p, float w, float h) const {
        aiVector3D v = trafo_ * aiVector3D(-delta_x_ * (1 - 2 * p.x / w),
                                           delta_y_ * (1 - 2 * p.y / h), -1);
        return {v.x, v.y, v.z};
    }

    /**
     * Convert 3d world coordinates to raster 2d coordinates.
     */
    aiVector2t<int> cam2raster(const Vec& p, float w, float h) const {
        // move to camera position and convert to camera space
        auto v = inverse_trafo_ * (aiVector3D(p.x, p.y, p.z) - mPosition);
        // project on z = 1 in normal camera space
        v.x /= v.z * -delta_x_;
        v.y /= v.z * -delta_y_;
        // rescale to image space
        int x = (v.x + 1) * w / 2;
        int y = (1 - v.y) * h / 2;
        return {x, y};
    }

private:
    aiMatrix3x3 trafo_;
    aiMatrix3x3 inverse_trafo_;
    float delta_x_, delta_y_;
};
