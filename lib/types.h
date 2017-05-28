#pragma once

#include "geometry.h"

#include <assimp/camera.h>
#include <assimp/types.h>

#include <array>
#include <assert.h>

using namespace turner;

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

enum class Axis : char { X = 0, Y = 1, Z = 2 };
static constexpr std::array<Axis, 3> AXES = {{Axis::X, Axis::Y, Axis::Z}};

/**
 * Color
 */
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
 * Light Source
 */
struct Light {
    Point3f position;
    Color color;
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
    Point3f raster2cam(const Vector2f& p, float w, float h) const {
        const auto v = trafo_ * aiVector3D(-delta_x_ * (1 - 2 * p.x / w),
                                           delta_y_ * (1 - 2 * p.y / h), -1);
        return {v.x, v.y, v.z};
    }

    /**
     * Convert 3d world coordinates to raster 2d coordinates.
     */
    Vector2i cam2raster(const Point3f& p, float w, float h) const {
        auto p_ai = aiVector3D(p.x, p.y, p.z);
        // move to camera position and convert to camera space
        auto v = inverse_trafo_ * (p_ai - mPosition);
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
