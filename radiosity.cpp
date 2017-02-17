#include "radiosity.h"
#include "config.h"
#include "lib/effects.h"
#include "lib/hierarchical.h"
#include "lib/matrix.h"
#include "lib/mesh.h"
#include "lib/output.h"
#include "lib/progress_bar.h"
#include "lib/radiosity.h"
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

#include <array>
#include <iostream>
#include <map>
#include <math.h>
#include <unordered_set>
#include <vector>

Color trace(const Vec& origin, const Vec& dir,
            KDTreeIntersection& tree_intersection,
            const std::vector<Color>& radiosity, const RadiosityConfig& conf) {
    Stats::instance().num_rays += 1;

    // intersection
    float dist_to_triangle, s, t;
    auto triangle_id =
        tree_intersection.intersect(aiRay{origin, dir}, dist_to_triangle, s, t);
    if (!triangle_id) {
        return conf.bg_color;
    }

    return radiosity[triangle_id];
}

Color trace(const Vec& origin, const Vec& dir,
            KDTreeIntersection& tree_intersection, const RadiosityMesh& mesh,
            const FaceRadiosityHandle& rad, const RadiosityConfig& conf) {
    Stats::instance().num_rays += 1;

    // intersection
    float dist_to_triangle, s, t;
    auto triangle_id =
        tree_intersection.intersect(aiRay{origin, dir}, dist_to_triangle, s, t);
    if (!triangle_id) {
        return conf.bg_color;
    }

    auto face = RadiosityMesh::FaceHandle(static_cast<size_t>(triangle_id));
    return mesh.property(rad, face);
}

Color trace_gouraud(const Vec& origin, const Vec& dir,
                    KDTreeIntersection& tree_intersection,
                    const RadiosityMesh& mesh,
                    const VertexRadiosityHandle& vrad,
                    const RadiosityConfig& conf) {
    Stats::instance().num_rays += 1;

    // intersection
    float dist_to_triangle, s, t;
    auto triangle_id =
        tree_intersection.intersect(aiRay{origin, dir}, dist_to_triangle, s, t);
    if (!triangle_id) {
        return conf.bg_color;
    }

    // TODO: Use vertices directly. Beware of ordering!
    CornerVertices corners;
    auto exists = mesh.get_property_handle(corners, "corner_vertices");
    assert(exists);
    UNUSED(exists);
    RadiosityMesh::FaceHandle face(static_cast<size_t>(triangle_id));
    const auto& vs = mesh.property(corners, face);

    // color interpolation
    const auto& rad_a = mesh.property(vrad, vs[0]);
    const auto& rad_b = mesh.property(vrad, vs[1]);
    const auto& rad_c = mesh.property(vrad, vs[2]);

    auto rad = (1 - s - t) * rad_a + s * rad_b + t * rad_c;
    rad.a = 1; // TODO
    return rad;
}

std::vector<Color> compute_radiosity(KDTree& tree) {
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

    KDTreeIntersection tree_intersection(tree);
    for (size_t i = 0; i < num_triangles; ++i) {
        // construct form factor matrix (F_ij)
        for (size_t j = i; j < num_triangles; ++j) {
            if (i == j) {
                F(i, i) = 0;
            } else {
                F(i, j) = form_factor(tree_intersection, i, j);
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

Image raycast(const KDTree& tree, const RadiosityConfig& conf,
              const Camera& cam, const std::vector<Color>& radiosity,
              Image&& image) {
    Runtime rt(Stats::instance().runtime_ms);

    std::cerr << "Rendering          ";

    ThreadPool pool(conf.num_threads);
    std::vector<std::future<void>> tasks;

    for (size_t y = 0; y < image.height(); ++y) {
        tasks.emplace_back(
            pool.enqueue([&image, &cam, &tree, &radiosity, y, &conf]() {
                // TODO: we need only one tree intersection per thread, not task
                KDTreeIntersection tree_intersection(tree);

                for (size_t x = 0; x < image.width(); ++x) {
                    auto cam_dir = cam.raster2cam(
                        aiVector2D(x, y), image.width(), image.height());

                    Stats::instance().num_prim_rays += 1;
                    image(x, y) += trace(cam.mPosition, cam_dir,
                                         tree_intersection, radiosity, conf);

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

    return image;
}

Image raycast(const KDTree& tree, const RadiosityConfig& conf,
              const Camera& cam, const RadiosityMesh& mesh, Image&& image) {
    Runtime rt(Stats::instance().runtime_ms);

    std::cerr << "Rendering          ";

    ThreadPool pool(conf.num_threads);
    std::vector<std::future<void>> tasks;

    FaceRadiosityHandle frad;
    bool exists = false;
    exists = mesh.get_property_handle(frad, "face_radiosity");
    assert(exists);
    VertexRadiosityHandle vrad;
    exists = mesh.get_property_handle(vrad, "vertex_radiosity");
    assert(exists);
    UNUSED(exists);

    for (size_t y = 0; y < image.height(); ++y) {
        tasks.emplace_back(pool.enqueue([&image, &cam, &tree, &mesh, &frad,
                                         &vrad, y, &conf]() {
            // TODO: we need only one tree intersection per thread, not task
            KDTreeIntersection tree_intersection(tree);

            for (size_t x = 0; x < image.width(); ++x) {
                auto cam_dir = cam.raster2cam(aiVector2D(x, y), image.width(),
                                              image.height());

                Stats::instance().num_prim_rays += 1;

                if (!conf.gouraud_enabled) {
                    image(x, y) += trace(cam.mPosition, cam_dir,
                                         tree_intersection, mesh, frad, conf);
                } else {
                    image(x, y) +=
                        trace_gouraud(cam.mPosition, cam_dir, tree_intersection,
                                      mesh, vrad, conf);
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

    return image;
}

Image render_feature_lines(const KDTree& tree, const RadiosityConfig& conf,
                           const Camera& cam, Image&& image) {
    // Render feature lines after
    // "Ray Tracing NPR-Style Feature Lines" by Choudhury and Parker.
    std::cerr << "Drawing mesh lines ";

    constexpr float offset = 1.f;
    constexpr std::array<Vec2, 8> offsets = {
        Vec2{0.f, 0.f},           Vec2{offset, 0.f},
        Vec2{offset, offset},     Vec2{0.f, offset},
        Vec2{0.f, offset / 2},    Vec2{offset / 2, offset},
        Vec2{offset, offset / 2}, Vec2{offset / 2, 0.f}};

    ThreadPool pool(conf.num_threads);
    std::vector<std::future<void>> mesh_tasks;

    for (size_t y = 0; y < image.height(); ++y) {
        mesh_tasks.emplace_back(pool.enqueue([&image, offsets, &cam, &tree, y,
                                              &conf]() {
            // TODO: we need only one tree intersection per thread, not task
            KDTreeIntersection tree_intersection(tree);

            for (size_t x = 0; x < image.width(); ++x) {
                float dist_to_triangle, s, t;
                std::unordered_set<KDTreeIntersection::OptionalId> triangle_ids;

                // Shoot center ray.
                auto cam_dir = cam.raster2cam(aiVector2D(x + 0.5f, y + 0.5f),
                                              image.width(), image.height());
                auto center_id = tree_intersection.intersect(
                    Ray(cam.mPosition, cam_dir), dist_to_triangle, s, t);
                triangle_ids.insert(center_id);

                // Sample disc rays around center.
                // TODO: Sample disc with Poisson or similar.
                for (auto offset : offsets) {
                    cam_dir =
                        cam.raster2cam(aiVector2D(x + offset[0], y + offset[1]),
                                       image.width(), image.height());
                    auto id = tree_intersection.intersect(
                        Ray(cam.mPosition, cam_dir), dist_to_triangle, s, t);
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

Image render_radiosity_mesh(const RadiosityMesh& mesh, const Camera& cam,
                            Image&& image) {
    auto draw_pixel = [&image](int x, int y) {
        if (0 <= x && static_cast<size_t>(x) < image.width() && 0 <= y &&
            static_cast<size_t>(y) < image.height()) {
            image(x, y) = Color(1, 1, 1, 1);
        }
    };

    auto draw_point = [](Image& image, int x, int y) {
        Color white(1, 1, 1, 1);
        image(x - 1, y - 1) = white;
        image(x - 1, y) = white;
        image(x - 1, y + 1) = white;
        image(x, y - 1) = white;
        image(x, y) = white;
        image(x, y + 1) = white;
        image(x + 1, y - 1) = white;
        image(x + 1, y) = white;
        image(x + 1, y + 1) = white;
    };

    size_t i = 0;
    for (auto it = mesh.faces_begin(); it != mesh.faces_end(); ++it) {
        const auto& fhandle = *it;

        for (const auto& halfedge : mesh.fh_range(fhandle)) {
            auto offset =
                RadiosityMesh::Point(0, 0, 0) * (0.25f * i / mesh.n_faces());
            auto from = mesh.point(mesh.from_vertex_handle(halfedge)) + offset;
            auto to = mesh.point(mesh.to_vertex_handle(halfedge)) + offset;
            auto from_raster = cam.cam2raster({from[0], from[1], from[2]},
                                              image.width(), image.height());
            auto to_raster = cam.cam2raster({to[0], to[1], to[2]},
                                            image.width(), image.height());
            bresenham(from_raster.x, from_raster.y, to_raster.x, to_raster.y,
                      draw_pixel);

            draw_point(image, from_raster.x, from_raster.y);
        }
        i++;
    }

    return image;
}

int main(int argc, char const* argv[]) {
    RadiosityConfig conf = RadiosityConfig::from_docopt(
        docopt::docopt(USAGE, {argv + 1, argv + argc}, true, "radiosity"));
    // import scene
    Assimp::Importer importer;
    const aiScene* scene =
        importer.ReadFile(conf.filename.c_str(),
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
    if (sceneCam.mAspect == 0) {
        sceneCam.mAspect = conf.aspect;
    } else {
        conf.aspect = sceneCam.mAspect;
    }
    auto* camNode = scene->mRootNode->FindNode(sceneCam.mName);
    assert(camNode != nullptr);
    const Camera cam(camNode->mTransformation, sceneCam);

    // Scene triangles
    auto triangles = triangles_from_scene(scene);
    Stats::instance().num_triangles = triangles.size();
    KDTree tree(std::move(triangles));

    // Image
    int width = conf.width;
    assert(width > 0);
    int height = width / cam.mAspect;
    Image image(width, height);

    // Compute radiosity
    std::vector<Color> radiosity;

    // TODO: split in functions
    if (conf.mode == RadiosityConfig::EXACT) {
        if (conf.verbose) {
            std::cerr << "Mode: exact" << std::endl;
            std::cerr << conf << std::endl;
        }

        radiosity = compute_radiosity(tree);
        image = raycast(tree, conf, cam, radiosity, std::move(image));
        if (conf.mesh == RadiosityConfig::SIMPLE_MESH) {
            image = render_mesh(tree.triangles(), cam, std::move(image));
        } else if (conf.mesh == RadiosityConfig::FEATURE_MESH) {
            image = render_feature_lines(tree, conf, cam, std::move(image));
        }
    } else if (conf.mode == RadiosityConfig::HIERARCHICAL) {
        conf.min_area = ::min(tree.triangles().begin(), tree.triangles().end(),
                              [](const Triangle& tri) { return tri.area(); });
        conf.min_area /= pow(4, conf.max_subdivisions);
        if (conf.verbose) {
            std::cerr << "Mode: hierarchical" << std::endl;
            std::cerr << conf << std::endl;
        } else {
            std::cerr << "Minimal area: " << conf.min_area << std::endl;
            std::cerr << "Form factor epsilon: " << conf.F_eps << std::endl;
            std::cerr << "Maximum iterations: " << conf.max_iterations
                      << std::endl;
            std::cerr << "Shooting radiosity epsilon: " << conf.BF_eps
                      << std::endl;
        }

        HierarchicalRadiosity model(tree, conf.F_eps, conf.min_area,
                                    conf.BF_eps, conf.max_iterations);
        try {
            model.compute();
        } catch (std::runtime_error e) {
            std::cerr << "Error: " << e.what() << std::endl;
            image = render_radiosity_mesh(model.mesh(), cam, std::move(image));
            std::cout << image << std::endl;
            return 1;
        }

        KDTree refined_tree(model.triangles());
        Stats::instance().num_triangles = refined_tree.num_triangles();
        Stats::instance().kdtree_height = refined_tree.height();

        if (conf.exact_hierarchical_enabled) {
            radiosity = compute_radiosity(refined_tree);
            image =
                raycast(refined_tree, conf, cam, radiosity, std::move(image));
        } else {
            image = raycast(refined_tree, conf, cam, model.mesh(),
                            std::move(image));
        }

        if (conf.mesh == RadiosityConfig::SIMPLE_MESH) {
            image = render_mesh(tree.triangles(), cam, std::move(image));
        } else if (conf.mesh == RadiosityConfig::FEATURE_MESH) {
            image = render_feature_lines(tree, conf, cam, std::move(image));
        }

        if (conf.links_enabled) {
            image = model.visualize_links(cam, std::move(image));
        }
    }

    // output stats
    std::cerr << Stats::instance() << std::endl;

    // output image
    std::cout << image << std::endl;

    return 0;
}
