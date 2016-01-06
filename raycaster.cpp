#include "lib/output.h"
#include "lib/intersection.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include <math.h>
#include <vector>
#include <iostream>


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


float ray_node_intersection(
    const aiRay& ray,
    const aiScene* scene, const aiNode& node)
{
    float min_r = -1;

    for (int i = 0; i < node.mNumMeshes; ++i) {
        const auto& mesh = *scene->mMeshes[node.mMeshes[i]];
        const auto& T = node.mTransformation;

        // std::cout << "Ray " << ray << std::endl;
        for (int i = 0; i < mesh.mNumFaces; ++i) {
            const auto& face = mesh.mFaces[i];
            assert(face.mNumIndices == 3);

            auto V0 = T * mesh.mVertices[face.mIndices[0]];
            auto V1 = T * mesh.mVertices[face.mIndices[1]];
            auto V2 = T * mesh.mVertices[face.mIndices[2]];

            // aiVector3D t(0, 0, 0);


            // TODO: hack

            auto r = ray_triangle_intersection(
                ray.pos, ray.pos + ray.dir,
                V0, V1, V2);

            // if (ray.dir.y == 0) {
            //     std::cout << "Face " << V0 << "; " << V1 << "; " << V2
            //               << " --> " << r << std::endl ;
            // }


            // std::cout << "r " << r << std::endl;

            if (r < 0) {
                continue;
            }

            if (min_r < 0 || r < min_r) {
                min_r = r;
            }
        }
    }

    return min_r;
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
        cam.mAspect = 16.f/9.f;
    }

    std::cerr << cam << std::endl;

    int height = width / cam.mAspect;

    // raycasting

    auto* camNode = scene->mRootNode->FindNode("Camera");
    assert(camNode != nullptr);
    const auto& CT = camNode->mTransformation;

    const auto& cube = scene->mRootNode->FindNode("Cube");
    assert(cube != nullptr);

    // TODO:
    // 1. Use blender directly (Done)
    // 2. Fix camera position (Done)
    // 3. Find better constants and interpolation of colors. (Done)
    // 4. Refactor code and move everything into functions.

    auto pos = CT * aiVector3D(0, 0, 0);
    std::vector<int> grayscale;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            auto dir = raster2cam(cam, aiVector2D(x, y), width, height);
            dir = (CT * dir - pos).Normalize();

            float dist = ray_node_intersection(
                aiRay(pos, std::move(dir)), scene, *cube);

            if (dist < 0) {
                grayscale.push_back(0);
            } else {
                grayscale.push_back(dist * 100.f);
            }
        }
    }

    // compute max and min values that are non-zero
    int min_nonzero = 0, max = 0;
    for (auto d : grayscale) {
        if (min_nonzero == 0 || (0 < d && d < min_nonzero)) {
            min_nonzero = d;
        }
        if (max == 0 || d > max) {
            max = d;
        }
    }

    // output image
    std::cout << "P2" << std::endl;
    std::cout << width << " " << height << std::endl;
    std::cout << (max - min_nonzero + 5) << std::endl;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            auto d = grayscale.at(y*width + x);
            if (d == 0) {
                std::cout << 0 << " ";
            } else {
                std::cout << max - d + 5 << " ";
            }
        }
        std::cout << std::endl;
    }

    return 0;
}
