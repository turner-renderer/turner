#include "lib/output.h"
#include "lib/intersection.h"

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


std::pair<float, const aiMesh*> ray_node_intersection(
    const aiRay& ray,
    const aiScene& scene, const aiNode& node)
{
    float min_r = -1;
    const aiMesh* min_mesh = nullptr;

    for (int i = 0; i < node.mNumMeshes; ++i) {
        const auto& mesh = *scene.mMeshes[node.mMeshes[i]];
        const auto& T = node.mTransformation;

        for (int i = 0; i < mesh.mNumFaces; ++i) {
            const auto& face = mesh.mFaces[i];
            assert(face.mNumIndices == 3);

            auto V0 = T * mesh.mVertices[face.mIndices[0]];
            auto V1 = T * mesh.mVertices[face.mIndices[1]];
            auto V2 = T * mesh.mVertices[face.mIndices[2]];

            auto r = ray_triangle_intersection(
                ray.pos, ray.pos + ray.dir,
                V0, V1, V2);

            if (r < 0) {
                continue;
            }

            if (min_r < 0 || r < min_r) {
                min_r = r;
                min_mesh = &mesh;
            }
        }
    }

    return std::make_pair(min_r, min_mesh);
}

std::pair<float, const aiMesh*> ray_nodes_intersection(
    const aiRay& ray,
    const aiScene& scene, const std::vector<aiNode*>& nodes)
{
    std::pair<float, const aiMesh*> min_rm{-1, nullptr};
    for (auto node : nodes) {
        auto rm = ray_node_intersection(ray, scene, *node);
        auto r = rm.first;
        if (r < 0) {
            continue;
        }

        if (min_rm.first < 0 || r < min_rm.first) {
            min_rm = std::move(rm);
        }
    }
    return min_rm;
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

    // raycasting

    auto* camNode = scene->mRootNode->FindNode("Camera");
    assert(camNode != nullptr);
    const auto& CT = camNode->mTransformation;
    std::cerr << "Cam Trafo: " << CT << std::endl;

    std::vector<aiNode*> geometry_nodes;
    for (int i = 0; i < scene->mRootNode->mNumChildren; ++i) {
        auto node = scene->mRootNode->mChildren[i];
        if (node->mNumMeshes > 0) {
            std::cerr << "DEBUG: " << "Adding new geometry node: "
                      << node->mName << std::endl;
            geometry_nodes.push_back(node);
        }
    }

    // TODO:
    // 1. Use blender directly (Done)
    // 2. Fix camera position (Done)
    // 3. Find better constants and interpolation of colors. (Done)
    // 4. Refactor code and move everything into functions.

    auto cam_pos = CT * aiVector3D(0, 0, 0);
    std::vector<aiColor4D> grayscale;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            auto cam_dir = raster2cam(cam, aiVector2D(x, y), width, height);
            cam_dir = (CT * cam_dir - cam_pos).Normalize();

            auto res = ray_nodes_intersection(
                aiRay(cam_pos, std::move(cam_dir)), *scene, geometry_nodes);
            float dist;
            const aiMesh* mesh;
            std::tie(dist, mesh) = res;

            if (dist < 0) {
                grayscale.emplace_back();  // black
            } else {
                grayscale.emplace_back();
                scene->mMaterials[mesh->mMaterialIndex]->Get(
                    AI_MATKEY_COLOR_DIFFUSE, grayscale.back());
                grayscale.back().a = dist;
            }
        }
    }

    // compute max and min values that are non-zero
    int min_nonzero = 0, max = 0;
    float max_channel = -1;
    for (auto color : grayscale) {
        int d = static_cast<int>(color.a);
        if (min_nonzero == 0 || (0 < d && d < min_nonzero)) {
            min_nonzero = d;
        }
        if (max == 0 || d > max) {
            max = d;
        }
        if (max_channel < -1) {
            max_channel = std::min(color.r, std::min(color.g, color.b));
        } else if (max_channel < color.r) {
            max_channel = color.r;
        } else if (max_channel < color.g) {
            max_channel = color.g;
        } else if (max_channel < color.b) {
            max_channel = color.b;
        }
    }
    std::cerr << max_channel << std::endl;
    max *= max_channel;
    int depth = max + (max - min_nonzero)/2;

    // output image
    std::cout << "P3" << std::endl;
    std::cout << width << " " << height << std::endl;
    std::cout << (depth - min_nonzero) << std::endl;
    int i = 0;
    for (auto color : grayscale) {
        if (i++ % width == 0) {
            std::cout << std::endl;
        }

        if (color.a == 0) {
            std::cout << "0 0 0 ";
        } else {
            int intensity = depth - static_cast<int>(color.a);
            std::cout << static_cast<int>(intensity * color.r) << " "
                      << static_cast<int>(intensity * color.g) << " "
                      << static_cast<int>(intensity * color.b) << " ";
        }
    }

    return 0;
}
