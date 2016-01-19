#pragma once

#include <assimp/types.h>
#include <assert.h>


using Vec = aiVector3D;
using SgnVec = aiVector3t<int>;


Vec operator/(int a, const Vec& v) {
    return {
        static_cast<float>(a)/v.x,
        static_cast<float>(a)/v.y,
        static_cast<float>(a)/v.z
    };
}


// AABB
struct Box {
    const Vec& operator[](size_t i) const {
        if (i == 0) {
            return min;
        } else if (i == 1) {
            return max;
        }
        assert(false);
    }

    Vec min;
    Vec max;
};


struct Ray : public aiRay {
    Ray(const Vec& pos, const Vec& dir)
        : aiRay(pos, dir)
        , invdir(1/dir)
    {
        sgn.x = (invdir.x < 0);
        sgn.y = (invdir.y < 0);
        sgn.z = (invdir.z < 0);
    }

    // pos, dir are in aiRay
    Vec invdir;
    SgnVec sgn;
};
