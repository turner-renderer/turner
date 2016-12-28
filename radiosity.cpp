#include "lib/radiosity.h"
#include "lib/hierarchical.h"
#include "lib/algorithm.h"
#include "lib/effects.h"
#include "lib/gauss_seidel.h"
#include "lib/lambertian.h"
#include "lib/matrix.h"
#include "lib/output.h"
#include "lib/range.h"
#include "lib/raster.h"
#include "lib/runtime.h"
#include "lib/sampling.h"
#include "lib/solid_angle.h"
#include "lib/stats.h"
#include "lib/triangle.h"
#include "lib/xorshift.h"
#include "trace.h"

#include <ThreadPool.h>
#include <assimp/Importer.hpp>  // C++ importer interface
#include <assimp/postprocess.h> // Post processing flags
#include <assimp/scene.h>       // Output data structure
#include <docopt/docopt.h>

#include <array>
#include <chrono>
#include <iostream>
#include <map>
#include <math.h>
#include <stack>
#include <unordered_set>
#include <vector>


std::array<Triangle, 6> subdivide6(const Triangle& tri) {
    const auto& a = tri.vertices[0];
    const auto& b = tri.vertices[1];
    const auto& c = tri.vertices[2];

    const auto& na = tri.normals[0];
    const auto& nb = tri.normals[1];
    const auto& nc = tri.normals[2];

    auto m = tri.midpoint();
    auto ma = (b + c) / 2.f;
    auto mb = (a + c) / 2.f;
    auto mc = (a + b) / 2.f;

    auto nm = tri.interpolate_normal(1.f / 3, 1.f / 3, 1.f / 3);
    auto nma = tri.interpolate_normal(0.f, 0.5f, 0.5f);
    auto nmb = tri.interpolate_normal(0.5f, 0.f, 0.5f);
    auto nmc = tri.interpolate_normal(0.5f, 0.5f, 0.f);

    auto copy_tri = [&tri](const Vec& a, const Vec& b, const Vec& c,
                           const Vec& na, const Vec& nb, const Vec& nc) {
        return Triangle{{a, b, c},       {na, nb, nc}, tri.ambient,
                        tri.diffuse,     tri.emissive, tri.reflective,
                        tri.reflectivity};
    };

    return {copy_tri(a, mc, m, na, nmc, nm), copy_tri(mc, b, m, nmc, nb, nm),
            copy_tri(b, ma, m, nb, nma, nm), copy_tri(ma, c, m, nma, nc, nm),
            copy_tri(c, mb, m, nc, nmb, nm), copy_tri(mb, a, m, nmb, na, nm)};
}

Color trace(const Vec& origin, const Vec& dir, const Tree& tree,
            const std::vector<Color>& radiosity, const Configuration& conf) {
    Stats::instance().num_rays += 1;

    // intersection
    float dist_to_triangle, s, t;
    auto triangle_id =
        tree.intersect(aiRay{origin, dir}, dist_to_triangle, s, t);
    if (!triangle_id) {
        return conf.bg_color;
    }

    return radiosity[triangle_id];
}

std::vector<Color> compute_radiosity(const Tree& tree) {
    using MatrixF = math::Matrix<float>;
    using VectorF = math::Vector<float>;
    size_t num_triangles = tree.num_triangles();

    // compute matrices
    MatrixF F(num_triangles, num_triangles);
    VectorF rho_r(num_triangles);
    VectorF rho_g(num_triangles);
    VectorF rho_b(num_triangles);
    VectorF E_r(num_triangles);
    VectorF E_g(num_triangles);
    VectorF E_b(num_triangles);

    for (size_t i = 0; i < num_triangles; ++i) {
        // construct form factor matrix (F_ij)
        for (size_t j = i; j < num_triangles; ++j) {
            if (i == j) {
                F(i, i) = 0;
            } else {
                F(i, j) = form_factor(tree, i, j);
                F(j, i) = tree[i].area() / tree[j].area() * F(i, j);
            }
        }

        const auto& triangle = tree[i];

        // construct material diagonal matrix (ρ_i)
        rho_r(i) = triangle.diffuse.r;
        rho_g(i) = triangle.diffuse.g;
        rho_b(i) = triangle.diffuse.b;

        // construct vector of emitters
        E_r(i) = triangle.emissive.r;
        E_g(i) = triangle.emissive.g;
        E_b(i) = triangle.emissive.b;
    }

    // solve radiosity equation with Gauß-Seidel Iteration
    MatrixF K_r(num_triangles, num_triangles);
    MatrixF K_g(num_triangles, num_triangles);
    MatrixF K_b(num_triangles, num_triangles);
    // I - rho.asDiagonal * F
    for (size_t r = 0; r < num_triangles; ++r) {
        for (size_t c = 0; c < num_triangles; ++c) {
            if (r != c) {
                K_r(r, c) = -rho_r(r) * F(r, c);
                K_g(r, c) = -rho_g(r) * F(r, c);
                K_b(r, c) = -rho_b(r) * F(r, c);
            } else {
                K_r(r, c) = 1.0f - rho_r(r) * F(r, c);
                K_g(r, c) = 1.0f - rho_g(r) * F(r, c);
                K_b(r, c) = 1.0f - rho_b(r) * F(r, c);
            }
        }
    }

    // We intialize B with emitter values.
    auto B_r = gauss_seidel(K_r, E_r, E_r, 10);
    auto B_g = gauss_seidel(K_g, E_g, E_g, 10);
    auto B_b = gauss_seidel(K_b, E_b, E_g, 10);

    // combine results in a vector
    std::vector<Color> B;
    for (size_t i = 0; i != num_triangles; ++i) {
        B.emplace_back(B_r(i) > 0 ? B_r(i) : 0, B_g(i) > 0 ? B_g(i) : 0,
                       B_b(i) > 0 ? B_b(i) : 0, 1.f);
    }
    return B;
}

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

            aiColor4D ambient, diffuse, reflective, emissive;
            material->Get(AI_MATKEY_COLOR_AMBIENT, ambient);
            material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuse);
            material->Get(AI_MATKEY_COLOR_REFLECTIVE, reflective);
            material->Get(AI_MATKEY_COLOR_EMISSIVE, emissive);

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

Image raycast(const KDTree& tree, const Configuration& conf, const Camera& cam,
              const std::vector<Color>& radiosity, Image&& image) {
    Runtime rt(Stats::instance().runtime_ms);

    std::cerr << "Rendering          ";

    ThreadPool pool(conf.num_threads);
    std::vector<std::future<void>> tasks;

    for (size_t y = 0; y < image.height(); ++y) {
        tasks.emplace_back(
            pool.enqueue([&image, &cam, &tree, &radiosity, y, &conf]() {
                for (size_t x = 0; x < image.width(); ++x) {
                    for (int i = 0; i < conf.num_pixel_samples; ++i) {
                        auto cam_dir = cam.raster2cam(
                            aiVector2D(x, y), image.width(), image.height());

                        Stats::instance().num_prim_rays += 1;
                        image(x, y) += trace(cam.mPosition, cam_dir, tree,
                                             radiosity, conf);
                    }

                    image(x, y) = exposure(image(x, y), conf.exposure);

                    // gamma correction
                    if (conf.gamma_correction_enabled) {
                        image(x, y) = gamma(image(x, y), conf.inverse_gamma);
                    }
                    // image(x, y) = Color(1, 1, 1, 1);
                }
            }));
    }

    long completed = 0;

    for (auto& task : tasks) {
        task.get();
        completed += 1;
        float progress = static_cast<float>(completed) / tasks.size();
        const int bar_width = progress * 20;
        std::cerr << "\rRendering          "
                  << "[" << std::string(bar_width, '-')
                  << std::string(20 - bar_width, ' ') << "] "
                  << std::setfill(' ') << std::setw(6) << std::fixed
                  << std::setprecision(2) << (progress * 100.0) << '%';
        std::cerr.flush();
    }
    std::cerr << std::endl;

    return image;
}

Image render_feature_lines(const KDTree& tree, const Configuration& conf,
                           const Camera& cam, Image&& image) {
    // Render feature lines after
    // "Ray Tracing NPR-Style Feature Lines" by Choudhury and Parker.
    std::cerr << "Drawing mesh lines ";

    ThreadPool pool(conf.num_threads);
    std::vector<std::future<void>> mesh_tasks;

    constexpr float offset = 1.f;
    constexpr std::array<Vec2, 8> offsets = {
        Vec2{0.f, 0.f},           Vec2{offset, 0.f},
        Vec2{offset, offset},     Vec2{0.f, offset},
        Vec2{0.f, offset / 2},    Vec2{offset / 2, offset},
        Vec2{offset, offset / 2}, Vec2{offset / 2, 0.f}};
    for (size_t y = 0; y < image.height(); ++y) {
        mesh_tasks.emplace_back(pool.enqueue([&image, offsets, &cam, &tree, y,
                                              &conf]() {
            for (size_t x = 0; x < image.width(); ++x) {
                float dist_to_triangle, s, t;
                std::unordered_set<KDTree::OptionalId> triangle_ids;

                // Shoot center ray.
                auto cam_dir = cam.raster2cam(aiVector2D(x + 0.5f, y + 0.5f),
                                              image.width(), image.height());
                auto center_id = tree.intersect(Ray(cam.mPosition, cam_dir),
                                                dist_to_triangle, s, t);
                triangle_ids.insert(center_id);

                // Sample disc rays around center.
                // TODO: Sample disc with Poisson or similar.
                for (auto offset : offsets) {
                    cam_dir =
                        cam.raster2cam(aiVector2D(x + offset[0], y + offset[1]),
                                       image.width(), image.height());
                    auto id = tree.intersect(Ray(cam.mPosition, cam_dir),
                                             dist_to_triangle, s, t);
                    triangle_ids.insert(id);
                }

                constexpr float M_2 = 0.5f * offsets.size();
                // All hit primitives except the one hit by center.
                const float m = triangle_ids.size() - 1.f;
                float e = std::pow(std::abs(m - M_2) / M_2, 10);
                image(x, y) = image(x, y) * e;
            }
        }));
    }

    size_t completed = 0;
    for (auto& task : mesh_tasks) {
        task.get();
        completed += 1;
        float progress = static_cast<float>(completed) / mesh_tasks.size();
        int bar_width = progress * 20;
        std::cerr << "\rDrawing mesh lines "
                  << "[" << std::string(bar_width, '-')
                  << std::string(20 - bar_width, ' ') << "] "
                  << std::setfill(' ') << std::setw(6) << std::fixed
                  << std::setprecision(2) << (progress * 100.0) << '%';
        std::cerr.flush();
    }
    std::cerr << std::endl;

    return image;
}

Image render_mesh(const Triangles& triangles, const Camera& cam,
                  Image&& image) {
    auto draw_pixel = [&image](int x, int y) {
        if (0 <= x && static_cast<size_t>(x) < image.width() && 0 <= y &&
            static_cast<size_t>(y) < image.height()) {
            image(x, y) = Color();
        }
    };

    for (const auto& tri : triangles) {
        const auto& a =
            cam.cam2raster(tri.vertices[0], image.width(), image.height());
        const auto& b =
            cam.cam2raster(tri.vertices[1], image.width(), image.height());
        const auto& c =
            cam.cam2raster(tri.vertices[2], image.width(), image.height());
        bresenham(a.x, a.y, b.x, b.y, draw_pixel);
        bresenham(b.x, b.y, c.x, c.y, draw_pixel);
        bresenham(c.x, c.y, a.x, a.y, draw_pixel);
    }

    return std::move(image);
}

static const char USAGE[] =
    R"(Usage: radiosity (direct|hierarchical) <filename> [options]

Options:
  -w --width=<px>                   Width of the image [default: 640].
  -a --aspect=<num>                 Aspect ratio of the image. If the model has
                                    specified the aspect ratio, it will be
                                    used. Otherwise default value is 1.
  --background=<3x float>           Background color [default: 0 0 0].
  -t --threads=<int>                Number of threads [default: 1].
  --inverse-gamma=<float>           Inverse of gamma for gamma correction
                                    [default: 0.454545].
  --no-gamma-correction             Disables gamma correction.
  -e --exposure=<float>             Exposure of the image. [default: 1.0]
  --rad-simple-mesh                 Render mesh without depth overlapping.
  --rad-features-mesh               Render mesh of features.
  --rad-links                       Render hierarchical radiosity links.
  --form-factor-eps=<float>         Link when form factor estimate is below [default: 0.04].
                                    Hierarchical radiosity only.
  --rad-shoot-eps=<float>           Refine link when shooting radiosity times
                                    form factor is too high [default: 1e-6].
  --max-subdivisions=<int>          Maximum number of subdivisions for smallest
                                    triangle [default: 3].
  --max-iterations=<int>            Maximum iterations to solve system
                                    [default: 1000].
)";

int main(int argc, char const* argv[]) {
    // parameters
    std::map<std::string, docopt::value> args =
        docopt::docopt(USAGE, {argv + 1, argv + argc}, true, "radiosity");

    const Configuration conf{1, 0, 1, 0 // unused configuration arguments
                             ,
                             args["--threads"].asLong(),
                             args["--background"].asString(),
                             std::stof(args["--inverse-gamma"].asString()),
                             args["--no-gamma-correction"].asBool(), 1,
                             std::stof(args["--exposure"].asString())};

    // import scene
    Assimp::Importer importer;
    const aiScene* scene =
        importer.ReadFile(args["<filename>"].asString().c_str(),
                          aiProcess_CalcTangentSpace | aiProcess_Triangulate |
                              aiProcess_JoinIdenticalVertices |
                              aiProcess_GenNormals | aiProcess_SortByPType);

    if (!scene) {
        std::cout << importer.GetErrorString() << std::endl;
        return 1;
    }

    // setup camera
    assert(scene->mNumCameras == 1); // we can deal only with a single camera
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

    // Scene triangles
    auto triangles = triangles_from_scene(scene);
    Stats::instance().num_triangles = triangles.size();
    KDTree tree(std::move(triangles));

    // Image
    int width = args["--width"].asLong();
    assert(width > 0);
    int height = width / cam.mAspect;
    Image image(width, height);

    // Compute radiosity
    std::vector<Color> radiosity;
    if (args["direct"].asBool()) {
        radiosity = compute_radiosity(tree);
        image = raycast(tree, conf, cam, radiosity, std::move(image));
        if (args["--rad-simple-mesh"] && args["--rad-simple-mesh"].asBool()) {
            image = render_mesh(tree.triangles(), cam, std::move(image));
        } else {
            image = render_feature_lines(tree, conf, cam, std::move(image));
        }
    } else if (args["hierarchical"].asBool()) {
        // compute minimal area
        int max_subdivisions = args["--max-subdivisions"].asLong();
        float min_area = ::min(tree.triangles().begin(), tree.triangles().end(),
                               [](const Triangle& tri) { return tri.area(); });
        min_area /= pow(4, max_subdivisions);
        std::cerr << "Minimal area: " << min_area << std::endl;

        float F_eps = std::stof(args["--form-factor-eps"].asString());
        std::cerr << "Form factor epsilon: " << F_eps << std::endl;

        size_t max_iterations = args["--max-iterations"].asLong();
        std::cerr << "Maximum iterations: " << max_iterations << std::endl;

        float BF_eps = std::stof(args["--rad-shoot-eps"].asString());
        std::cerr << "Shooting radiosity epsilon: " << BF_eps << std::endl;

        HierarchicalRadiosity model(tree, F_eps, min_area, max_iterations, BF_eps);
        auto triangles_with_rad = model.compute();
        KDTree refined_tree(std::move(triangles_with_rad.first));
        radiosity = triangles_with_rad.second;

        image = raycast(refined_tree, conf, cam, radiosity, std::move(image));

        if (args["--rad-simple-mesh"] && args["--rad-simple-mesh"].asBool()) {
            image =
                render_mesh(refined_tree.triangles(), cam, std::move(image));
        } else if (args["--rad-features-mesh"] &&
                   args["--rad-features-mesh"].asBool()) {
            image =
                render_feature_lines(refined_tree, conf, cam, std::move(image));
        }

        if (args["--rad-links"] && args["--rad-links"].asBool()) {
            image = model.visualize_links(cam, std::move(image));
        }
    } else {
        assert(!"parsed arguments");
        throw std::logic_error("parsed arguments");
    }

    // output stats
    std::cerr << Stats::instance() << std::endl;

    // output image
    std::cout << image << std::endl;

    return 0;
}
