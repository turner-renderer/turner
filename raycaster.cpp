#include "lib/output.h"
#include "lib/intersection.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include <math.h>
#include <iostream>


aiVector3D raster2cam(
    const aiCamera& cam, const aiVector2D& p,
    const int w, const int h)
{
    float delta_x = tan(cam.mHorizontalFOV / 2.);
    float delta_y = delta_x / cam.mAspect;
    return aiVector3D(
        -delta_x * (1 - 2 * p.x / (float)w),
        delta_y * (1 - 2 * p.y / (float)h),
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

    assert(scene->mNumCameras == 1);

    const auto& cam = *scene->mCameras[0];
    assert(cam.mPosition == aiVector3D(0, 0, 0));
    assert(cam.mUp == aiVector3D(0, 1, 0));
    assert(cam.mLookAt == aiVector3D(0, 0, -1));

    int height = width / cam.mAspect;

    // raycasting

    // auto& RT = scene->mRootNode->mTransformation;

    auto* camNode = scene->mRootNode->FindNode("Camera");
    assert(camNode != nullptr);
    const auto& CT = camNode->mTransformation;
    std::cerr << CT << std::endl;

    const auto& cube = scene->mRootNode->FindNode("Cube");
    assert(cube != nullptr);

    std::cout << "P2" << std::endl;

    // debug
    auto lt = CT * raster2cam(cam, aiVector2D(0, 0), width, height);
    std::cout << "# " << lt << std::endl;

    // TODO:
    // 1. Use blender directly
    // 2. Fix camera position
    // 3. Find better constants and interpolation of colors.

    std::cout << width << " " << height << std::endl;
    std::cout << 800 << std::endl;

    auto pos = CT * aiVector3D(0, 0, 0);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            auto dir = raster2cam(cam, aiVector2D(x, y), width, height);
            dir = (CT * dir).Normalize();

            float dist = ray_node_intersection(
                aiRay(pos, std::move(dir)), scene, *cube) * 100;

            if (dist < 0) {
                std::cout << 0 << " ";
            } else {
                int d = dist;
                if (d < 500) {
                    std::cout << 0 <<  " ";
                } else {
                    std::cout << 1000 - ((d - 500) * 2) << " ";
                }
            }
        }
        std::cout << std::endl;
    }

    return 0;
}
