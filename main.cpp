#include "trace.h"
#include "lib/output.h"
#include "lib/image.h"
#include "lib/range.h"
#include "lib/runtime.h"
#include "lib/triangle.h"
#include "lib/stats.h"
#include "lib/xorshift.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags
#include <docopt.h>
#include <ThreadPool.h>

#include <math.h>
#include <vector>
#include <map>
#include <iostream>


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
            const auto& material = scene->mMaterials[mesh.mMaterialIndex];

            aiColor4D ambient, diffuse, reflective;
            material->Get(AI_MATKEY_COLOR_AMBIENT, ambient);
            material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuse);
            material->Get(AI_MATKEY_COLOR_REFLECTIVE, reflective);

            float reflectivity = 0.f;
            material->Get(AI_MATKEY_REFLECTIVITY, reflectivity);

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
                    diffuse,
                    reflective,
                    reflectivity
                });
            }
        }
    }
    return triangles;
}

static const char USAGE[] =
R"(Usage: raytracer <filename> [options]

Options:
  -w --width=<px>                   Width of the image [default: 640].
  -a --aspect=<num>                 Aspect ratio of the image. If the model has
                                    specified the aspect ratio, it will be
                                    used. Otherwise default value is 1.
  -d --max-depth=<int>              Maximum recursion depth for raytracing
                                    [default: 3].
  --shadow=<float>                  Intensity of shadow [default: 0.5].
  --background=<3x float>           Background color [default: 0 0 0].
  -p --pixel-samples=<int>          Number of samples per pixel [default: 1].
  -m --monte-carlo-samples=<int>    Monto Carlo samples per ray [default: 8].
                                    Used only in pathtracer.
  -t --threads=<int>                Number of threads [default: 1].
  --inverse-gamma=<float>           Inverse of gamma for gamma correction
                                    [default: 0.454545].
  --no-gamma-correction             Disables gamma correction.
)";

int main(int argc, char const *argv[])
{
    // parameters
    std::map<std::string, docopt::value> args =
        docopt::docopt(USAGE, {argv + 1, argv + argc}, true, "raytracer 0.2");

    const Configuration conf { args["--max-depth"].asLong()
                             , std::stof(args["--shadow"].asString())
                             , args["--pixel-samples"].asLong()
                             , args["--monte-carlo-samples"].asLong()
                             , args["--threads"].asLong()
                             , args["--background"].asString()
                             , std::stof(args["--inverse-gamma"].asString())
                             , args["--no-gamma-correction"].asBool()
                             };

    // import scene
    std::cerr << "Loading scene..." << std::endl;
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

    // setup light
    // we can deal only with one single or no light at all
    assert(scene->mNumLights == 0 || scene->mNumLights == 1);
    std::vector<Light> lights;
    if(scene->mNumLights == 1) {
        auto& rawLight = *scene->mLights[0];

        auto* lightNode = scene->mRootNode->FindNode(rawLight.mName);
        assert(lightNode != nullptr);
        const auto& LT = lightNode->mTransformation;
        lights.push_back(
            { LT * aiVector3D()
            , aiColor4D
                { rawLight.mColorDiffuse.r
                , rawLight.mColorDiffuse.g
                , rawLight.mColorDiffuse.b
                , 1
                }
            });
    }

    // load triangles from the scene into a kd-tree
    std::cerr << "Loading triangles and building kd-tree..." << std::endl;
    Runtime loading_time;
    auto triangles = triangles_from_scene(scene);
    Stats::instance().num_triangles = triangles.size();
    Tree tree(std::move(triangles));
    Stats::instance().loading_time_ms = loading_time();
    Stats::instance().kdtree_height = tree.height();

    //
    // Raytracer
    //

    int width = args["--width"].asLong();
    assert(width > 0);
    int height = width / cam.mAspect;

    Image image(width, height);
    {
        Runtime rt(Stats::instance().runtime_ms);

        std::cerr << "Rendering ";

        ThreadPool pool(conf.num_threads);
        std::vector<std::future<void>> tasks;

        for (int y = 0; y < height; ++y) {
            tasks.emplace_back(pool.enqueue([
                    &image, &cam, &tree, &lights, width, height, y, &conf]()
            {
                float dx, dy;
                xorshift64star<float> gen(42);

                for (int x = 0; x < width; ++x) {
                    for (int i = 0; i < conf.num_pixel_samples; ++i) {
                        dx = gen();
                        dy = gen();

                        auto cam_dir = cam.raster2cam(
                            aiVector2D(x + dx, y + dy), width, height);

                        Stats::instance().num_prim_rays += 1;
                        image(x, y) += trace(cam.mPosition, cam_dir, tree,
                            lights, 0, conf);
                    }
                    image(x, y) /=
                        static_cast<float>(conf.num_pixel_samples);

                    // gamma correction
                    if (conf.gamma_correction_enabled) {
                        image(x, y).r = powf(image(x, y).r, conf.inverse_gamma);
                        image(x, y).g = powf(image(x, y).g, conf.inverse_gamma);
                        image(x, y).b = powf(image(x, y).b, conf.inverse_gamma);
                    }
                }
            }));
        }

        long completed = 0;

        for (auto& task: tasks) {
            task.get();
            completed += 1;
            float progress = static_cast<float>(completed) / tasks.size();
            int bar_width = progress * 20;
            std::cerr
                << "\rRendering "
                << "[" << std::string(bar_width, '-')
                << std::string(20 - bar_width, ' ') << "] "
                << std::setfill(' ') << std::setw(6)
                << std::fixed << std::setprecision(2) << (progress * 100.0) << '%';
            std::cerr.flush();
        }
        std::cerr << std::endl;
    }

    // output stats
    std::cerr << Stats::instance() << std::endl;

    // output image
    std::cout << image << std::endl;
    return 0;
}
