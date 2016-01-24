#include "lib/output.h"
#include "lib/image.h"
#include "lib/lambertian.h"
#include "lib/range.h"
#include "lib/runtime.h"
#include "lib/triangle.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include <math.h>
#include <vector>
#include <iostream>
#include <chrono>


ssize_t ray_intersection(const aiRay& ray, const Triangles& triangles,
        float& min_r, float& min_s, float& min_t) {

    ssize_t triangle_index = -1;
    min_r = -1;
    float r, s, t;
    for (size_t i = 0; i < triangles.size(); i++) {
        auto intersect = triangles[i].intersect(ray, r, s, t);
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


Triangles triangles_from_scene(const aiScene* scene) {
    Triangles triangles;
    for (auto node : make_range(
            scene->mRootNode->mChildren, scene->mRootNode->mNumChildren))
    {
        const auto& T = node->mTransformation;
        if (node->mNumMeshes == 0) {
            continue;
        }

        for (auto mesh_index : make_range(node->mMeshes, node->mNumMeshes)) {
            const auto& mesh = *scene->mMeshes[mesh_index];

            aiColor4D ambient, diffuse;
            scene->mMaterials[mesh.mMaterialIndex]->Get(
                AI_MATKEY_COLOR_AMBIENT, ambient);
            scene->mMaterials[mesh.mMaterialIndex]->Get(
                AI_MATKEY_COLOR_DIFFUSE, diffuse);

            for (aiFace face : make_range(mesh.mFaces, mesh.mNumFaces)) {
                assert(face.mNumIndices == 3);
                triangles.push_back(Triangle{
                    // vertices
                    {{
                        T * mesh.mVertices[face.mIndices[0]],
                        T * mesh.mVertices[face.mIndices[1]],
                        T * mesh.mVertices[face.mIndices[2]]
                    }},
                    // normals
                    {{
                        T * mesh.mNormals[face.mIndices[0]],
                        T * mesh.mNormals[face.mIndices[1]],
                        T * mesh.mNormals[face.mIndices[2]]
                    }},
                    ambient,
                    diffuse
                });
            }
        }
    }
    return triangles;
}



int main(int argc, char const *argv[])
{
    if (!(1 < argc && argc < 4)) {
        std::cerr << "Usage: " << argv[0] << " <filename> [width]\n";
        return 0;
    }

    // parameters
    std::string filename(argv[1]);
    int width = argc == 3 ? std::atoi(argv[2]) : 640;
    assert(width > 0);

    // import scene
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

    // setup camera
    assert(scene->mNumCameras == 1);  // we can deal only with a single camera
    auto& sceneCam = *scene->mCameras[0];
    auto* camNode = scene->mRootNode->FindNode(sceneCam.mName);
    assert(camNode != nullptr);

    const Camera cam(camNode->mTransformation, sceneCam);
    std::cerr << "Camera" << std::endl;
    std::cerr << cam << std::endl;

    int height = width / cam.mAspect;

    // setup light
    assert(scene->mNumLights == 1);  // we can deal only with a single light
    auto& light = *scene->mLights[0];

    std::cerr << "Light" << std::endl;
    std::cerr << "Diffuse: " << light.mColorDiffuse << std::endl;

    auto* lightNode = scene->mRootNode->FindNode(light.mName);
    assert(lightNode != nullptr);
    const auto& LT = lightNode->mTransformation;
    std::cerr << "Light Trafo: " << LT << std::endl;
    auto light_pos = LT * aiVector3D();
    auto light_color = aiColor4D{
        light.mColorDiffuse.r,
        light.mColorDiffuse.g,
        light.mColorDiffuse.b,
        1};

    // load triangles from the scene
    auto triangles = triangles_from_scene(scene);

    //
    // Raytracer
    //

    Image image(width, height);
    {
        auto rt = Runtime(std::cerr, "Rendering time: ");
        float r, s, t;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // intersection
                auto cam_dir = cam.raster2cam(aiVector2D(x, y), width, height);
                auto triangle_index = ray_intersection(
                    aiRay(cam.mPosition, cam_dir), triangles, r, s, t);
                if (triangle_index < 0) {
                    image(x, y) = aiColor4D() + 0.4f;
                    continue;
                }

                // light direction
                auto p = cam.mPosition + r * cam_dir;
                auto light_dir = (light_pos - p).Normalize();

                // interpolate normal
                const auto& triangle = triangles[triangle_index];
                const auto& n0 = triangle.normals[0];
                const auto& n1 = triangle.normals[1];
                const auto& n2 = triangle.normals[2];
                auto normal = ((1.f - s - t)*n0 + s*n1 + t*n2).Normalize();

                image(x, y) += lambertian(
                    light_dir, normal, triangle.diffuse, light_color);
                image(x, y) += triangle.ambient;
            }
        }
    }

    // output image
    std::cout << image << std::endl;
    return 0;
}
