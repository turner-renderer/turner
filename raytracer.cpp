#include "lib/output.h"
#include "lib/image.h"
#include "lib/lambertian.h"
#include "lib/range.h"
#include "lib/runtime.h"
#include "lib/triangle.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags
#include <docopt.h>

#include <math.h>
#include <vector>
#include <map>
#include <iostream>
#include <chrono>


struct Configuration {
    int max_depth;
    float shadow_intensity;

    void check() {
        assert(0 < max_depth);
        assert(0 <= shadow_intensity && shadow_intensity <= 1);
    }
};


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
        if (node->mNumMeshes == 0) {
            continue;
        }

        const auto& T = node->mTransformation;
        const aiMatrix3x3 Tp(T);  // trafo without translation

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
                        Tp * mesh.mNormals[face.mIndices[0]],
                        Tp * mesh.mNormals[face.mIndices[1]],
                        Tp * mesh.mNormals[face.mIndices[2]]
                    }},
                    ambient,
                    diffuse
                });
            }
        }
    }
    return triangles;
}

aiColor4D trace(const aiVector3D origin, const aiVector3D dir,
        const Triangles triangles, const aiVector3D light_pos,
        const aiColor4D light_color, int depth, const Configuration& conf)
{
    auto result = aiColor4D(0, 0, 0, 1);

    if (depth > conf.max_depth) {
        return result;
    }

    // intersection
    float dist_to_triangle, s, t;
    auto triangle_index = ray_intersection(
        aiRay(origin, dir), triangles,
        dist_to_triangle, s, t);
    if (triangle_index < 0) {
        return result;
    }

    // light direction
    auto p = origin + dist_to_triangle * dir;
    auto light_dir = (light_pos - p).Normalize();

    // interpolate normal
    const auto& triangle = triangles[triangle_index];
    auto normal = triangle.interpolate_normal(1.f - s - t, s, t);

    // direct light
    result += 0.9f * lambertian(
        light_dir, normal, triangle.diffuse, light_color);

    // move slightly in direction of normal
    auto p2 = p + normal * 0.0001f;

    // reflection
    // compute reflected ray from incident ray
    auto reflected_ray_dir = dir - 2.f * (normal * dir) * normal;
    auto reflected_color = triangle.diffuse * 0.1f * trace(
        p2, reflected_ray_dir,
        triangles, light_pos, light_color, depth + 1, conf);
    result += reflected_color;

    // shadow
    float dist_to_next_triangle;
    light_dir = (light_pos - p2).Normalize();
    float dist_to_light = (light_pos - p2).Length();
    auto shadow_triangle = ray_intersection(
        aiRay(p2, light_dir),
        triangles,
        dist_to_next_triangle, s, t);

    if (shadow_triangle >= 0 && dist_to_next_triangle < dist_to_light) {
        result *= 1.f - conf.shadow_intensity;
    }

    return result;
}


static const char USAGE[] =
R"(Usage: raytracer <filename> [options]

Options:
  -w --width=<px>           Width of the image [default: 640].
  -a --aspect=<num>         Aspect ratio of the image. If the model has
                            specified the aspect ratio, it will be used.
                            Otherwise default value is 1.
  --max-depth=<int>         Maximum recursion depth for raytracing [default: 3].
  --shadow=<float>          Intensity of shadow [default: 0.5].
)";

int main(int argc, char const *argv[])
{
    // parameters
    std::map<std::string, docopt::value> args =
        docopt::docopt(USAGE, {argv + 1, argv + argc}, true, "raytracer 0.2");

    Configuration conf;
    conf.max_depth = args["--max-depth"].asLong();
    conf.shadow_intensity = std::stof(args["--shadow"].asString());
    conf.check();

    // import scene
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(
        args["<filename>"].asString().c_str(),
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
    if (args["--aspect"]) {
        sceneCam.mAspect = std::stof(args["--aspect"].asString());
        assert(sceneCam.mAspect > 0);
    } else if (sceneCam.mAspect == 0) {
        sceneCam.mAspect = 1.f;
    }
    auto* camNode = scene->mRootNode->FindNode(sceneCam.mName);
    assert(camNode != nullptr);

    const Camera cam(camNode->mTransformation, sceneCam);
    std::cerr << "Camera" << std::endl;
    std::cerr << cam << std::endl;

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

    int width = args["--width"].asLong();
    assert(width > 0);
    int height = width / cam.mAspect;

    Image image(width, height);
    {
        auto rt = Runtime(std::cerr, "Rendering time: ");

        std::cerr << "Rendering ";
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                auto cam_dir = cam.raster2cam(aiVector2D(x, y), width, height);

                image(x, y) = trace(cam.mPosition, cam_dir, triangles,
                        light_pos, light_color, 0, conf);
            }

            // update progress bar
            auto height_fraction = height / 20;
            if (y % height_fraction == 0) {
                std::cerr << ".";
            }
        }
        std::cerr << std::endl;
    }

    // output image
    std::cout << image << std::endl;
    return 0;
}
