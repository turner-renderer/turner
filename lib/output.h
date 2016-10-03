#pragma once

#include "image.h"
#include "triangle.h"
#include "types.h"
#include "stats.h"

#include <assimp/scene.h>

#include <ostream>
#include <iomanip>


inline std::ostream& operator<<(std::ostream& os, const aiVector3D& v) {
    return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}

inline std::ostream& operator<<(std::ostream& os, const aiColor3D& c) {
    return os << "Color(" << c.r << ", " << c.g << ", " << c.b << ")";
}

inline std::ostream& operator<<(std::ostream& os, const aiColor4D& c) {
    return os << "Color(" << c.r << ", " << c.g << ", " << c.b << ", " << c.a << ")";
}

inline std::ostream& operator<<(std::ostream& os, const aiMatrix4x4& mat) {
    os << "[" << mat.a1 << " " << mat.a2 << " " << mat.a3 << " " << mat.a4 << "]\n";
    os << "[" << mat.b1 << " " << mat.b2 << " " << mat.b3 << " " << mat.b4 << "]\n";
    os << "[" << mat.c1 << " " << mat.c2 << " " << mat.c3 << " " << mat.c4 << "]\n";
    os << "[" << mat.d1 << " " << mat.d2 << " " << mat.d3 << " " << mat.d4 << "]";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const aiMatrix3x3& mat) {
    os << "[" << mat.a1 << " " << mat.a2 << " " << mat.a3 << "]\n";
    os << "[" << mat.b1 << " " << mat.b2 << " " << mat.b3 << "]\n";
    os << "[" << mat.c1 << " " << mat.c2 << " " << mat.c3 << "]\n";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const aiString& str) {
    return os << str.C_Str();
}

inline std::ostream& operator<<(std::ostream& os, const Triangle& tri) {
    return os << "Triangle("
        << "vertices=[" << tri.vertices[0] << " "
                        << tri.vertices[1] << " "
                        << tri.vertices[2]
        << "])";
}

inline std::ostream& operator<<(std::ostream& os, const aiNode& node) {
    os << "Node(name=" << node.mName;

    if (node.mNumChildren > 0) {
        os << ", children=[";
        for (size_t i = 0; i < node.mNumChildren; ++i) {
            os << *node.mChildren[i];
            if (i + 1 != node.mNumChildren) {
                os << ", ";
            }
        }
        os << "]";
    }
    return os << ")";
}

inline std::ostream& operator<<(std::ostream& os, const aiMesh& mesh) {
    for (size_t i = 0; i < mesh.mNumFaces; ++i) {
        const auto& face = mesh.mFaces[i];
        os << "---" << std::endl;
        for (size_t j = 0; j < face.mNumIndices; ++j) {
            os << mesh.mVertices[face.mIndices[j]] << std::endl;
        }
    }
    return os;
}


inline std::ostream& operator<<(std::ostream& os, const aiRay& ray) {
    return os << ray.pos << " + t * " << ray.dir;
}


inline std::ostream& operator<<(std::ostream& os, const aiCamera& cam) {
    return os << "Camera("
        << "mAspect=" << cam.mAspect << " "
        << "mClipPlaneFar=" << cam.mClipPlaneFar << " "
        << "mClipPlaneNear=" << cam.mClipPlaneNear << " "
        << "mHorizontalFOV=" << cam.mHorizontalFOV << " "
        << "mLookAt=" << cam.mLookAt << " "
        << "mPosition=" << cam.mPosition << " "
        << "mUp=" << cam.mUp << ")";
}

//
// Output image in PBM format.
//
// Cf. https://en.wikipedia.org/wiki/Netpbm_format#PPM_example.
//
inline std::ostream& operator<<(std::ostream& os, const Image& img) {
    os << "P3" << std::endl;
    os << img.width << " " << img.height << std::endl;
    os << 255 << std::endl;
    int i = 0;
    for (auto color : img) {
        if (i++ % img.width == 0) {
            os << std::endl;
        }

        os << std::setfill(' ') << std::setw(3)
           << static_cast<int>(clamp(255 * color.r * color.a)) << " "
           << std::setfill(' ') << std::setw(3)
           << static_cast<int>(clamp(255 * color.g * color.a)) << " "
           << std::setfill(' ') << std::setw(3)
           << static_cast<int>(clamp(255 * color.b * color.a)) << " ";
    }

    return os;
}


inline std::ostream& operator<<(std::ostream& os, const Box& box) {
    return os << "Box[" << box.min << ", " << box.max << "]";
}

inline std::ostream& operator<<(std::ostream& os, const Stats& stats) {
    return os
        << "Triangles      : " << stats.num_triangles << std::endl
        << "Kd-Tree Height : " << stats.kdtree_height << std::endl
        << "Rays           : " << stats.num_rays << std::endl
        << "Rays (primary) : " << stats.num_prim_rays << std::endl
        << "Rays/sec       : "
        << 1000 * stats.num_rays / stats.runtime_ms << std::endl
        << "Loading time   : " << 1.0 * stats.loading_time_ms / 1000 << " sec"
            << std::endl
        << "Rendering time : " << 1.0 * stats.runtime_ms / 1000 << " sec";
}
