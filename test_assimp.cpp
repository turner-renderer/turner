#include "lib/intersection.h"
#include "lib/output.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include <iostream>


int main(int argc, char const *argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <filename>" << std::endl;
        return 0;
    }

    std::string filename(argv[1]);

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

    std::cout << "Cameras:" << std::endl;
    for (size_t i = 0; i < scene->mNumCameras; ++i) {
        auto camera = scene->mCameras[i];
        std::cout << "Camera name: " << camera->mName << std::endl;
        std::cout << "Look at vector: " << camera->mLookAt << std::endl;
    }

    std::cout << "Hierarchie" << std::endl;
    std::cout << *scene->mRootNode << std::endl;
    std::cout << scene->mRootNode->mTransformation << std::endl;

    auto cube = scene->mRootNode->FindNode("Cube");
    if (cube != nullptr) {
        std::cout << "List faces of the cube" << std::endl;
        for (size_t i = 0; i < cube->mNumMeshes; ++i) {
            std::cout << *scene->mMeshes[cube->mMeshes[i]] << std::endl;
        }

        std::cout << cube->mTransformation << std::endl;
    }

    auto nodeCam = scene->mRootNode->FindNode("Camera");
    if (nodeCam != nullptr) {
        std::cout << nodeCam->mTransformation << std::endl;
    }

    if (scene->mNumCameras > 0) {
        auto camera = scene->mCameras[0];
        std::cout << "Aspect: " << camera->mAspect << std::endl;
        std::cout << "mClipPlaneFar: " << camera->mClipPlaneFar << std::endl;
        std::cout << "mClipPlaneNear: " << camera->mClipPlaneNear << std::endl;
        std::cout << "mHorizontalFOV: " << camera->mHorizontalFOV << std::endl;
        std::cout << "mLookAt: " << camera->mLookAt << std::endl;
        std::cout << "mPosition: " << camera->mPosition << std::endl;
        std::cout << "mUp: " << camera->mUp << std::endl;
    }

    return 0;
}
