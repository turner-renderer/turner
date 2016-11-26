#include "lib/radiosity.h"
#include "lib/effects.h"
#include "lib/gauss_seidel.h"
#include "lib/image.h"
#include "lib/lambertian.h"
#include "lib/matrix.h"
#include "lib/output.h"
#include "lib/range.h"
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
#include <unordered_set>
#include <vector>

struct Linknode;

struct Quadnode {
    bool operator==(const Quadnode&) const { return true; }
    bool is_leaf() const {
        // due to full subdivision: any child is none -> all children are none
        return children[0] == nullptr;
    }

    float rad_gather; // gathering radiosity
    float rad_shoot;  // shooting radiosity
    float emission;
    float area;
    float rho; // reflectivity
    std::array<std::unique_ptr<Quadnode>, 4> children;
    std::unique_ptr<Linknode>& first_gathering_link;
};

struct Linknode {
    Quadnode* q;                    // gathering node
    Quadnode* p;                    // shooting node
    float F_qp;                     // form factor from q to p
    std::unique_ptr<Linknode> next; // next gathering link of node q
};

bool oracle(const Quadnode&, const Quadnode&, float) { return false; }

enum class SubdivideRes { LEFT, RIGHT, NONE };
SubdivideRes subdivide(Quadnode&, Quadnode&) { return SubdivideRes::NONE; }

void link(Quadnode&, Quadnode&){};

void refine(Quadnode& p, Quadnode& q, float eps) {
    if (oracle(p, q, eps)) {
        link(p, q);
    } else {
        auto to_subdivide = subdivide(p, q);
        if (to_subdivide == SubdivideRes::LEFT) {
            for (auto& child : p.children) {
                refine(p, *child, eps);
            }
        } else if (to_subdivide == SubdivideRes::RIGHT) {
            for (auto& child : q.children) {
                refine(*child, q, eps);
            }
        } else {
            link(p, q);
        }
    }
}

void gather_radiosity(Quadnode& p) {
    p.rad_gather = 0;
    Linknode* link = p.first_gathering_link.get();
    while (link) {
        p.rad_gather += p.rho * link->F_qp * link->q->rad_shoot;
        link = link->next.get();
    }

    if (p.is_leaf()) {
        return;
    }

    for (auto& child : p.children) {
        gather_radiosity(*child);
    }
}

float push_pull_radiosity(Quadnode& p, float rad_down) {
    if (p.is_leaf()) {
        p.rad_shoot = p.emission + p.rad_gather + rad_down;
    } else {
        float rad_up = 0;
        for (auto& child : p.children) {
            float rad = push_pull_radiosity(*child, p.rad_gather + rad_down);
            rad_up += rad * child->area / p.area;
        }
        p.rad_shoot = rad_up;
    }
    return p.rad_shoot;
}

void solve_system(std::vector<Quadnode> surfaces) {
    while (true) // TODO: need convergence criteria
    {
        for (auto& p : surfaces) {
            gather_radiosity(p);
        }
        for (auto& p : surfaces) {
            push_pull_radiosity(p, 0);
        }
    }
}

// -----------------------------------------------------------------------------

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

static const char USAGE[] =
    R"(Usage: raytracer <filename> [options]

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
  --exposure=<float>                Exposure of the image. [default: 1.0]
)";

int main(int argc, char const* argv[]) {
    // parameters
    std::map<std::string, docopt::value> args =
        docopt::docopt(USAGE, {argv + 1, argv + argc}, true, "raytracer 0.2");

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

    // load triangles from the scene into a kd-tree
    auto raw_triangles = triangles_from_scene(scene);
    Triangles triangles;
    triangles.reserve(6 * raw_triangles.size());
    for (const auto& tri : raw_triangles) {
        auto sub_tris = subdivide6(tri);
        triangles.push_back(std::get<0>(sub_tris));
        triangles.push_back(std::get<1>(sub_tris));
        triangles.push_back(std::get<2>(sub_tris));
        triangles.push_back(std::get<3>(sub_tris));
        triangles.push_back(std::get<4>(sub_tris));
        triangles.push_back(std::get<5>(sub_tris));
    }

    Stats::instance().num_triangles = triangles.size();
    Tree tree(std::move(triangles));
    assert(tree.num_triangles() == Stats::instance().num_triangles);

    // compute radiosity
    auto radiosity = compute_radiosity(tree);

    //
    // Raycaster
    //

    int width = args["--width"].asLong();
    assert(width > 0);
    int height = width / cam.mAspect;

    Image image(width, height);
    {
        Runtime rt(Stats::instance().runtime_ms);

        std::cerr << "Rendering          ";

        ThreadPool pool(conf.num_threads);
        std::vector<std::future<void>> tasks;

        for (int y = 0; y < height; ++y) {
            tasks.emplace_back(pool.enqueue([&image, &cam, &tree, &radiosity,
                                             width, height, y, &conf]() {
                for (int x = 0; x < width; ++x) {
                    for (int i = 0; i < conf.num_pixel_samples; ++i) {
                        auto cam_dir =
                            cam.raster2cam(aiVector2D(x, y), width, height);

                        Stats::instance().num_prim_rays += 1;
                        image(x, y) += trace(cam.mPosition, cam_dir, tree,
                                             radiosity, conf);
                    }

                    image(x, y) = exposure(image(x, y), conf.exposure);

                    // gamma correction
                    if (conf.gamma_correction_enabled) {
                        image(x, y) = gamma(image(x, y), conf.inverse_gamma);
                    }
                }
            }));
        }

        long completed = 0;

        for (auto& task : tasks) {
            task.get();
            completed += 1;
            float progress = static_cast<float>(completed) / tasks.size();
            int bar_width = progress * 20;
            std::cerr << "\rRendering          "
                      << "[" << std::string(bar_width, '-')
                      << std::string(20 - bar_width, ' ') << "] "
                      << std::setfill(' ') << std::setw(6) << std::fixed
                      << std::setprecision(2) << (progress * 100.0) << '%';
            std::cerr.flush();
        }
        std::cerr << std::endl;

        // Render feature lines after
        // "Ray Tracing NPR-Style Feature Lines" by Choudhury and Parker.
        std::cerr << "Drawing mesh lines ";
        std::vector<std::future<void>> mesh_tasks;
        constexpr float offset = 1.f;
        constexpr std::array<Vec2, 8> offsets = {
            Vec2{0.f, 0.f},           Vec2{offset, 0.f},
            Vec2{offset, offset},     Vec2{0.f, offset},
            Vec2{0.f, offset / 2},    Vec2{offset / 2, offset},
            Vec2{offset, offset / 2}, Vec2{offset / 2, 0.f}};
        for (int y = 0; y < height; ++y) {
            mesh_tasks.emplace_back(pool.enqueue([&image, offsets, &cam, &tree,
                                                  width, height, y, &conf]() {
                for (int x = 0; x < width; ++x) {
                    float dist_to_triangle, s, t;
                    std::unordered_set<Tree::OptionalId> triangle_ids;

                    // Shoot center ray.
                    auto cam_dir = cam.raster2cam(
                        aiVector2D(x + 0.5f, y + 0.5f), width, height);
                    auto center_id = tree.intersect(Ray(cam.mPosition, cam_dir),
                                                    dist_to_triangle, s, t);
                    triangle_ids.insert(center_id);

                    // Sample disc rays around center.
                    // TODO: Sample disc with Poisson or similar.
                    for (auto offset : offsets) {
                        cam_dir = cam.raster2cam(
                            aiVector2D(x + offset[0], y + offset[1]), width,
                            height);
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

        completed = 0;
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
    }

    // output stats
    std::cerr << Stats::instance() << std::endl;

    // output image
    std::cout << image << std::endl;
    return 0;
}
