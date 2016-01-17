#include "image.h"

#include <assimp/scene.h>

#include <ostream>


std::ostream& operator<<(std::ostream& os, const aiVector3D& v) {
    return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}

std::ostream& operator<<(std::ostream& os, const aiColor4D& c) {
    return os << "Color(" << c.r << ", " << c.g << ", " << c.b << ")";
}

std::ostream& operator<<(std::ostream& os, const aiMatrix4x4& mat) {
    os << "[" << mat.a1 << " " << mat.a2 << " " << mat.a3 << " " << mat.a4 << "]\n";
    os << "[" << mat.b1 << " " << mat.b2 << " " << mat.b3 << " " << mat.b4 << "]\n";
    os << "[" << mat.c1 << " " << mat.c2 << " " << mat.c3 << " " << mat.c4 << "]\n";
    os << "[" << mat.d1 << " " << mat.d2 << " " << mat.d3 << " " << mat.d4 << "]";
    return os;
}

std::ostream& operator<<(std::ostream& os, const aiString& str) {
    return os << str.C_Str();
}

std::ostream& operator<<(std::ostream& os, const aiNode& node) {
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

std::ostream& operator<<(std::ostream& os, const aiMesh& mesh) {
    for (size_t i = 0; i < mesh.mNumFaces; ++i) {
        const auto& face = mesh.mFaces[i];
        os << "---" << std::endl;
        for (size_t j = 0; j < face.mNumIndices; ++j) {
            os << mesh.mVertices[face.mIndices[j]] << std::endl;
        }
    }
    return os;
}


std::ostream& operator<<(std::ostream& os, const aiRay& ray) {
    return os << ray.pos << " + t * " << ray.dir;
}


std::ostream& operator<<(std::ostream& os, const aiCamera& cam) {
    return os << "Camera("
        << "mAspect=" << cam.mAspect << " "
        << "mClipPlaneFar=" << cam.mClipPlaneFar << " "
        << "mClipPlaneNear=" << cam.mClipPlaneNear << " "
        << "mHorizontalFOV=" << cam.mHorizontalFOV << " "
        << "mLookAt=" << cam.mLookAt << " "
        << "mPosition=" << cam.mPosition << " "
        << "mUp=" << cam.mUp << ")";
}

std::ostream& operator<<(std::ostream& os, const Image img) {
    os << "P3" << std::endl;
    os << img.width << " " << img.height << std::endl;
    os << 255 << std::endl;
    int i = 0;
    for (auto color : img) {
        if (i++ % img.width == 0) {
            os << std::endl;
        }

        os << static_cast<int>(255 * color.r * color.a) << " "
        << static_cast<int>(255 * color.g * color.a) << " "
        << static_cast<int>(255 * color.b * color.a) << " ";
    }

    return os;
}
