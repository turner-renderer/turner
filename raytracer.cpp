#include "lib/output.h"
#include "lib/image.h"
#include "lib/intersection.h"
#include "lib/lambertian.h"
#include "lib/range.h"
#include "lib/types.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include <math.h>
#include <vector>
#include <iostream>

//
// Convert 2d raster coodinates into 3d cameras coordinates.
//
// We assume that the camera is trivial:
//   assert(cam.mPosition == aiVector3D(0, 0, 0));
//   assert(cam.mUp == aiVector3D(0, 1, 0));
//   assert(cam.mLookAt == aiVector3D(0, 0, -1));
//   assert(cam.mAspect != 0) {
//
// The positioning of the camera is done in its parent's node transformation
// matrix.
//
aiVector3D raster2cam(
    const aiCamera& cam, const aiVector2D& p,
    const int w, const int h)
{
    float delta_x = tan(cam.mHorizontalFOV / 2.);
    float delta_y = delta_x / cam.mAspect;
    return aiVector3D(
        -delta_x * (1 - 2 * p.x / static_cast<float>(w)),
        delta_y * (1 - 2 * p.y / static_cast<float>(h)),
        -1);
}


ssize_t ray_intersection(const aiRay& ray, const Triangles& triangles,
        float& min_r, float& min_s, float& min_t) {

    ssize_t triangle_index = -1;
    min_r = -1;
    float r, s, t;
    for (size_t i = 0; i < triangles.size(); i++) {
        auto intersect = ray_triangle_intersection(
            ray, triangles[i], r, s, t);
        if (intersect) {
            if (triangle_index < 0 || r < min_r) {
                min_r = r;
                min_s = s;
                min_t = t;
                triangle_index = i;
            }
        }
    }

    return triangle_index;
}

int main(int argc, char const *argv[])
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <filename> <width>\n";
        return 0;
    }

    std::string filename(argv[1]);
    int width = std::atoi(argv[2]);
    assert(width > 0);

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filename,
        aiProcess_CalcTangentSpace       |
        aiProcess_Triangulate            |
        aiProcess_JoinIdenticalVertices  |
        aiProcess_GenNormals             |
        aiProcess_SortByPType);

    if (!scene) {
        std::cout << importer.GetErrorString() << std::endl;
        return 1;
    }

    std::cerr << *scene->mRootNode << std::endl;

    assert(scene->mNumCameras == 1);
    auto& cam = *scene->mCameras[0];
    assert(cam.mPosition == aiVector3D(0, 0, 0));
    assert(cam.mUp == aiVector3D(0, 1, 0));
    assert(cam.mLookAt == aiVector3D(0, 0, -1));
    if (cam.mAspect == 0) {
        // cam.mAspect = 16.f/9.f;
        cam.mAspect = 1.f;
    }

    std::cerr << cam << std::endl;

    int height = width / cam.mAspect;

    // raytracing

    auto* camNode = scene->mRootNode->FindNode("Camera");
    assert(camNode != nullptr);
    const auto& CT = camNode->mTransformation;
    std::cerr << "Cam Trafo: " << CT << std::endl;
    auto cam_pos = CT * aiVector3D(0, 0, 0);

    // Get light node
    auto* lightNode = scene->mRootNode->FindNode("Light");
    assert(lightNode != nullptr);
    const auto& LT = lightNode->mTransformation;
    std::cerr << "Light Trafo: " << LT << std::endl;
    auto light_pos = LT * aiVector3D();

    // FIXME: Our scene does not have any lights yet.
    // std::cerr << scene->mNumLights << std::endl;
    // assert(scene->mNumLights == 1);
    // auto* light = scene->mLights[0];
    // TODO: ...
    auto light_color = aiColor4D(0xFF / 255., 0xF8 / 255., 0xDD / 255., 1);

    std::vector<Triangle> triangles;
    for (auto node :
            make_range(scene->mRootNode->mChildren, scene->mRootNode->mNumChildren))
    {
        const auto& T = node->mTransformation;
        if (node->mNumMeshes == 0) {
            continue;
        }

        for (auto mesh_index : make_range(node->mMeshes, node->mNumMeshes)) {
            const auto& mesh = *scene->mMeshes[node->mMeshes[mesh_index]];

            aiColor4D diffuse;
            scene->mMaterials[mesh.mMaterialIndex]->Get(
                AI_MATKEY_COLOR_DIFFUSE, diffuse);

            for (aiFace face : make_range(mesh.mFaces, mesh.mNumFaces)) {
                assert(face.mNumIndices == 3);
                auto v0 = T * mesh.mVertices[face.mIndices[0]];
                auto v1 = T * mesh.mVertices[face.mIndices[1]];
                auto v2 = T * mesh.mVertices[face.mIndices[2]];

                auto n0 = T * mesh.mNormals[face.mIndices[0]];
                auto n1 = T * mesh.mNormals[face.mIndices[1]];
                auto n2 = T * mesh.mNormals[face.mIndices[2]];

                triangles.push_back( { {v0, v1, v2}, {n0, n1, n2}, diffuse } );
            }
        }
    }

    // TODO:
    // 1. Better triangle structure
    // 2. Add ambient light.
    // 3. Find a nice scene (cornell box).

    Image image(width, height);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            auto cam_dir = raster2cam(cam, aiVector2D(x, y), width, height);
            cam_dir = (CT * cam_dir - cam_pos).Normalize();

            float r, s, t;
            auto triangle_index = ray_intersection(
                aiRay(cam_pos, std::move(cam_dir)), triangles, r, s, t);

            //image_data.emplace_back();  // black
            if (triangle_index < 0) {
                continue;
            }

            // intersection point
            auto p = cam_pos + r * cam_dir;

            const auto& triangle = triangles[triangle_index];

            // compute normal

            // barycentric coordinates
            float u = s;
            float v = t;
            float w = 1.f - u - v;

            auto aN = triangle.normals[0];
            auto bN = triangle.normals[1];
            auto cN = triangle.normals[2];
            auto normal = (w*aN + u*bN + v*cN).Normalize();

            // compute vector towards light
            auto light_dir = (light_pos - p).Normalize();

            image(x, y) = lambertian(light_dir, normal, triangle.diffuse, light_color);

            // TODO: get material's (ambient) color
        }
    }

    // output image
    std::cout << image << std::endl;

    return 0;
}
