#include "lib/intersection.h"
#include "lib/output.h"
#include "lib/range.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags
#include <assimp/DefaultLogger.hpp>     // Post processing flags

#include <iostream>


int main(int argc, char const *argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <filename>" << std::endl;
        return 0;
    }

    std::string filename(argv[1]);

    // Assimp::DefaultLogger::get()->setLogSeverity(Assimp::VERBOSE);
    // aiEnableVerboseLogging(true);

    Assimp::DefaultLogger::get()->setLogSeverity(Assimp::Logger::VERBOSE);

    // if (!DefaultLogger::isNullLogger()) {
    //     DefaultLogger::get()->setLogSeverity((d == AI_TRUE ? Logger::VERBOSE : Logger::NORMAL));
    // }
    // gVerboseLogging = d;

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filename,
        // aiProcess_CalcTangentSpace       |
        aiProcess_Triangulate            |
        // aiProcess_JoinIdenticalVertices  |
        aiProcess_GenNormals);
        // aiProcess_SortByPType);

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

    std::cout << "Lights: " << scene->mNumLights << std::endl;
    for (size_t i = 0; i < scene->mNumLights; ++i) {
        auto light = scene->mLights[i];
        std::cout << "Light name: " << light->mName << std::endl;
        std::cout << "Position: " << light->mPosition << std::endl;
        std::cout << "Color (Spec): " << light->mColorSpecular << std::endl;

        auto lightNode = scene->mRootNode->FindNode(light->mName);
        assert(lightNode);
        std::cout << "Light trafo: " << lightNode->mTransformation << std::endl;
    }

    std::cout << "Hierarchie" << std::endl;
    std::cout << *scene->mRootNode << std::endl;
    std::cout << scene->mRootNode->mTransformation << std::endl;

    for (int i = 0; i < scene->mRootNode->mNumChildren; ++i) {
        const auto& node = *scene->mRootNode->mChildren[i];
        std::cout << "\nList faces of " << node.mName << std::endl;
        for (int j = 0; j < node.mNumMeshes; ++j) {
            const auto& mesh = *scene->mMeshes[node.mMeshes[j]];

            aiString name;
            scene->mMaterials[mesh.mMaterialIndex]->Get(AI_MATKEY_NAME, name);
            std::cout << "Material: " << name << std::endl;

            for (int k = 0; k < mesh.mNumFaces; ++k) {
                std::cout << "Face" << std::endl;
                const auto& face = mesh.mFaces[k];
                for (int idx = 0; idx < face.mNumIndices; ++idx) {
                    std::cout << mesh.mVertices[face.mIndices[idx]] << " ";
                    std::cout << "n=" << mesh.mNormals[face.mIndices[idx]] << "\n";
                }

                // if (mesh.mColors[0] != nullptr) {
                //     for (int idx = 0; idx < face.mNumIndices; ++idx) {
                //         std::cout << mesh.mColors[0][face.mIndices[idx]]
                //                   << " ";
                //     }
                // }
                std::cout << std::endl;
            }

            // aiString name;
            // scene->mMaterials[mesh.mMaterialIndex]->Get(AI_MATKEY_NAME, name);
            // std::cout << name << std::endl;

            // aiColor4D color;
            // scene->mMaterials[mesh.mMaterialIndex]->Get(
            //     AI_MATKEY_COLOR_DIFFUSE, color);
            // std::cout << color << std::endl;
            // scene->mMaterials[mesh.mMaterialIndex]->Get(
            //     AI_MATKEY_COLOR_AMBIENT, color);
            // std::cout << color << std::endl;
            // scene->mMaterials[mesh.mMaterialIndex]->Get(
            //     AI_MATKEY_COLOR_SPECULAR, color);
            // std::cout << color << std::endl;
            // scene->mMaterials[mesh.mMaterialIndex]->Get(
            //     AI_MATKEY_COLOR_EMISSIVE, color);
            // std::cout << color << std::endl;
            // scene->mMaterials[mesh.mMaterialIndex]->Get(
            //     AI_MATKEY_COLOR_TRANSPARENT, color);
            // std::cout << color << std::endl;
            // scene->mMaterials[mesh.mMaterialIndex]->Get(
            //     AI_MATKEY_COLOR_REFLECTIVE, color);
            // std::cout << color << std::endl;

        }

        std::cout << "Trafo: " << node.mTransformation << std::endl;
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

    std::cout << "Materials" << std::endl;
    std::cout << &(*scene->mMaterials)[0] << std::endl;
    std::cout << &(*scene->mMaterials)[scene->mNumMaterials] << std::endl;
    std::cout << scene->mNumMaterials << std::endl;

    for (const aiMaterial* material :
            make_range(scene->mMaterials, scene->mNumMaterials))
    {
        // std::cout << &material << std::end;

        aiString name;
        material->Get(AI_MATKEY_NAME, name);
        std::cout << name << std::endl;

        aiColor4D color;
        material->Get(AI_MATKEY_COLOR_DIFFUSE, color);
        std::cout << "Diffuse " << color << std::endl;
        material->Get(AI_MATKEY_COLOR_AMBIENT, color);
        std::cout << "Ambient " << color << std::endl;
        material->Get(AI_MATKEY_COLOR_SPECULAR, color);
        std::cout << "Specular " << color << std::endl;
        material->Get(AI_MATKEY_COLOR_EMISSIVE, color);
        std::cout << "Emissive " << color << std::endl;
        material->Get(AI_MATKEY_COLOR_TRANSPARENT, color);
        std::cout << color << std::endl;
        material->Get(AI_MATKEY_COLOR_REFLECTIVE, color);
        std::cout << color << std::endl;

        // for (int j = 0; j < material.mNumProperties; ++j) {
        //     const auto& prop = *material.mProperties[j];
        //     std::cerr << prop.mKey << " ";
        // }
        // std::cerr << std::endl;

        // aiColor4D color;
        // material.Get(AI_MATKEY_COLOR_DIFFUSE, color);
        // std::cerr << color << std::endl;
        // material.Get(AI_MATKEY_COLOR_EMISSIVE, color);
        // std::cerr << color << std::endl;
        // material.Get(AI_MATKEY_COLOR_SPECULAR, color);
        // std::cerr << color << std::endl;

        // std::cerr << std::endl;
    }

    return 0;
}
