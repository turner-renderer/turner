#include "lib/effects.h"
#include "lib/output.h"
#include "lib/progress_bar.h"
#include "lib/range.h"
#include "lib/raster.h"
#include "lib/runtime.h"
#include "lib/stats.h"
#include "lib/triangle.h"
#include "lib/xorshift.h"
#include "trace.h"

#include <ThreadPool.h>
#include <assimp/Importer.hpp>  // C++ importer interface
#include <assimp/postprocess.h> // Post processing flags
#include <assimp/scene.h>       // Output data structure
#include <docopt/docopt.h>

#include <iostream>
#include <map>
#include <math.h>
#include <vector>

#include <mms/vector.h>
#include <mms/writer.h>

#include <fcntl.h>
#include <fstream>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/types.h>

Triangles triangles_from_scene(const aiScene* scene) {
    Triangles triangles;
    for (auto node : make_range(scene->mRootNode->mChildren,
                                scene->mRootNode->mNumChildren)) {
        if (node->mNumMeshes == 0) {
            continue;
        }

        const auto& T = node->mTransformation;
        const aiMatrix3x3 Tp(T); // trafo without translation

        for (auto mesh_index : make_range(node->mMeshes, node->mNumMeshes)) {
            const auto& mesh = *scene->mMeshes[mesh_index];
            const auto& material = scene->mMaterials[mesh.mMaterialIndex];

            aiColor4D ambient, diffuse, emissive, reflective;
            material->Get(AI_MATKEY_COLOR_AMBIENT, ambient);
            material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuse);
            material->Get(AI_MATKEY_COLOR_DIFFUSE, emissive);
            material->Get(AI_MATKEY_COLOR_REFLECTIVE, reflective);

            float reflectivity = 0.f;
            material->Get(AI_MATKEY_REFLECTIVITY, reflectivity);

            for (aiFace face : make_range(mesh.mFaces, mesh.mNumFaces)) {
                assert(face.mNumIndices == 3);
                triangles.push_back(
                    Triangle{// vertices
                             {{T * mesh.mVertices[face.mIndices[0]],
                               T * mesh.mVertices[face.mIndices[1]],
                               T * mesh.mVertices[face.mIndices[2]]}},
                             // normals
                             {{Tp * mesh.mNormals[face.mIndices[0]],
                               Tp * mesh.mNormals[face.mIndices[1]],
                               Tp * mesh.mNormals[face.mIndices[2]]}},
                             ambient,
                             diffuse,
                             emissive,
                             reflective,
                             reflectivity});
            }
        }
    }
    return triangles;
}

// Defined in the file with the trace implementation for the corresponding
// renderer.
extern const char* USAGE;

int main(int argc, char const* argv[]) {
    std::map<std::string, docopt::value> args =
        docopt::docopt(USAGE, {argv + 1, argv + argc});
    TracerConfig conf = TracerConfig::from_docopt(args);
    if (conf.verbose) {
        std::cerr << conf << std::endl;
    }

    // import scene
    std::cerr << "Loading scene..." << std::endl;
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(
        conf.filename, aiProcess_CalcTangentSpace | aiProcess_Triangulate |
                           aiProcess_JoinIdenticalVertices |
                           aiProcess_GenNormals | aiProcess_SortByPType);

    if (!scene) {
        std::cout << importer.GetErrorString() << std::endl;
        return 1;
    }

    // setup camera
    assert(scene->mNumCameras == 1); // we can deal only with a single camera
    auto& sceneCam = *scene->mCameras[0];
    if (sceneCam.mAspect > 0) {
        conf.aspect = sceneCam.mAspect;
    } else if (sceneCam.mAspect == 0) {
        sceneCam.mAspect = conf.aspect;
    }
    auto* camNode = scene->mRootNode->FindNode(sceneCam.mName);
    assert(camNode != nullptr);
    const Camera cam(camNode->mTransformation, sceneCam);

    // setup light
    // we can deal only with one single or no light at all
    assert(scene->mNumLights == 0 || scene->mNumLights == 1);
    std::vector<Light> lights;
    if (scene->mNumLights == 1) {
        auto& rawLight = *scene->mLights[0];

        auto* lightNode = scene->mRootNode->FindNode(rawLight.mName);
        assert(lightNode != nullptr);
        const auto& LT = lightNode->mTransformation;
        lights.push_back(
            {LT * aiVector3D(),
             aiColor4D{rawLight.mColorDiffuse.r, rawLight.mColorDiffuse.g,
                       rawLight.mColorDiffuse.b, 1}});
    }

    // load triangles from the scene into a kd-tree
    std::cerr << "Loading triangles and building kd-tree..." << std::endl;
    Runtime loading_time;
    auto triangles = triangles_from_scene(scene);
    Stats::instance().num_triangles = triangles.size();

    // Build and allocate tree data outside of tree

    std::vector<detail::TriangleId> ids(triangles.size(), 0);

    // Compute the bounding box of all triangles and fill in vector of all ids.
    auto box = triangles.front().bbox();
    for (size_t i = 1; i < triangles.size(); ++i) {
        box = box + triangles[i].bbox();
        ids[i] = i;
    }

    KDTreeBuildAlgorithm algo(triangles);
    mms::vector<mms::Standalone, detail::FlatNodeT<mms::Standalone>> tmp_nodes =
        flatten(std::unique_ptr<detail::TreeNode>(algo.build(std::move(ids), box)));

    // Save nodes and mmap them.

    // Serialize
    std::ofstream out("kdtree_cache");
    size_t pos = mms::write(out, tmp_nodes);
    out.close();

    // mmap data
    int fd = ::open("kdtree_cache", O_RDONLY);
    struct stat st;
    fstat(fd, &st);
    char* data = (char*) mmap(0, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
    auto nodes_ = reinterpret_cast<const mms::vector<mms::Mmapped, detail::FlatNode>*>(data + pos);

    KDTree tree(std::move(triangles), std::move(box), nodes_);
    Stats::instance().loading_time_ms = loading_time();
    Stats::instance().kdtree_height = tree.height();

    //
    // Raytracer
    //

    int width = conf.width;
    int height = width / cam.mAspect;

    Image image(width, height);
    {
        Runtime rt(Stats::instance().runtime_ms);

        std::cerr << "Rendering ";

        ThreadPool pool(conf.num_threads);
        std::vector<std::future<void>> tasks;

        for (int y = 0; y < height; ++y) {
            tasks.emplace_back(pool.enqueue([&image, &cam, &tree, &lights,
                                             width, height, y, &conf]() {
                // TODO: we need only one tree intersection per thread, not task
                KDTreeIntersection tree_intersection(tree);

                float dx, dy;
                xorshift64star<float> gen(42);

                for (int x = 0; x < width; ++x) {
                    for (int i = 0; i < conf.num_pixel_samples; ++i) {
                        dx = gen();
                        dy = gen();

                        auto cam_dir = cam.raster2cam(
                            aiVector2D(x + dx, y + dy), width, height);

                        Stats::instance().num_prim_rays += 1;
                        image(x, y) +=
                            trace(cam.mPosition, cam_dir, tree_intersection,
                                  lights, 0, conf);
                    }
                    image(x, y) /= static_cast<float>(conf.num_pixel_samples);

                    image(x, y) = exposure(image(x, y), conf.exposure);

                    // gamma correction
                    if (conf.gamma_correction_enabled) {
                        image(x, y) = gamma(image(x, y), conf.inverse_gamma);
                    }
                }
            }));
        }

        long completed = 0;
        auto progress_bar = ProgressBar(std::cerr, "Rendering", tasks.size());
        for (auto& task : tasks) {
            task.get();
            completed += 1;
            progress_bar.update(completed);
        }
        std::cerr << std::endl;
    }

    // output stats
    std::cerr << Stats::instance() << std::endl;

    // output image
    std::cout << image << std::endl;
    return 0;
}
