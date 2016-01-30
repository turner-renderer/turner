#pragma once

#include <assimp/types.h>
#include <assimp/camera.h>
#include <assert.h>
#include <sstream>
#include <vector>


using Vec = aiVector3D;

Vec operator/(int a, const Vec& v) {
    return {
        static_cast<float>(a)/v.x,
        static_cast<float>(a)/v.y,
        static_cast<float>(a)/v.z
    };
}


// AABB
struct Box {
    Vec min, max;
};


// Ray with precomputed inverse direction
struct Ray : public aiRay {
    Ray(const Vec& pos, const Vec& dir)
        : aiRay(pos, dir)
        , invdir(1/dir)
    {}

    Ray(const aiRay& ray) : Ray(ray.pos, ray.dir) {}

    // pos, dir are in aiRay
    Vec invdir;
};


// Represents a camera including transformation
class Camera : public aiCamera {
public:
    template<typename... Args>
    Camera(const aiMatrix4x4& trafo, Args... args)
        : aiCamera{args...}
        , trafo_(trafo)  // discard tranlation in trafo
    {
        assert(mPosition == Vec());
        assert(mUp == Vec(0, 1, 0));
        assert(mLookAt == Vec(0, 0, -1));
        assert(mAspect != 0);

        mPosition = trafo * mPosition;
        delta_x_ = tan(mHorizontalFOV);
        delta_y_ = delta_x_ / mAspect;
    }

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
    aiVector3D raster2cam(const aiVector2D& p, const int w, const int h) const
    {
        return trafo_ * aiVector3D(
            -delta_x_ * (1 - 2 * p.x / static_cast<float>(w)),
            delta_y_ * (1 - 2 * p.y / static_cast<float>(h)),
            -1);
    }

private:
    aiMatrix3x3 trafo_;
    float delta_x_, delta_y_;
};


std::vector<std::string> split(const std::string& s, char delim) {
    std::vector<std::string> tokens;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        if (!item.empty()) {
            tokens.push_back(item);
        }
    }
    return tokens;
}

aiColor4D parse_color4(const std::string& str) {
    const auto tokens = split(str, ' ');
    assert(tokens.size() == 4);
    return
        { std::stof(tokens[0])
        , std::stof(tokens[1])
        , std::stof(tokens[2])
        , std::stof(tokens[3])
        };
}