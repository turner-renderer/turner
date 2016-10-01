#include "helper.h"
#include "../lib/kdtree.h"
#include "../lib//output.h"
#include "../lib/runtime.h"
#include <catch.hpp>

#include <iostream>
#include <unordered_set>


TEST_CASE("Trivial smoke test", "[kdtree]")
{
    auto tri = random_triangle();
    KDTree kdtree({tri});
}

TEST_CASE("Smoke test", "[kdtree]")
{
    auto a = test_triangle({-1, -1, 0}, {1, -1, 0}, {1, 1, 0});
    auto b = test_triangle({0, 0, 0}, {1, 0, 0}, {1, 2, 0});
    KDTree tree(Triangles{a, b});
    REQUIRE(tree.height() == 0);
}

TEST_CASE("Four separated triangles test", "[kdtree]")
{
    auto a = test_triangle({0, 0, 1}, {0, 1, 1}, {1, 0, 1});
    auto b = test_triangle({2, 0, 1}, {3, 0, 1}, {3, 1, 1});
    auto c = test_triangle({0, 2, 1}, {0, 3, 1}, {1, 3, 1});
    auto d = test_triangle({3, 2, 1}, {3, 3, 1}, {2, 3, 1});
    KDTree tree(Triangles{a, b, c, d});
    REQUIRE(tree.height() == 1);
    REQUIRE(tree.size() == 4);

    KDTree::OptionalId hit;
    float r, s, t;

    hit = tree.intersect({{0, 0, 0}, {0.5f, 0.5f, 1}}, r, s, t);
    REQUIRE(hit);
    REQUIRE(tree.at(hit) == a);
    REQUIRE(r == 1);
    REQUIRE(s == 0.5f);
    REQUIRE(t == 0.5f);

    hit = tree.intersect({{0, 0, 0}, {2.5f, 0.5f, 1}}, r, s, t);
    REQUIRE(hit);
    REQUIRE(tree.at(hit) == b);
    REQUIRE(r == 1);
    REQUIRE(s == 0);
    REQUIRE(t == 0.5f);

    hit = tree.intersect({{0, 0, 0}, {0.5f, 2.5f, 1}}, r, s, t);
    REQUIRE(hit);
    REQUIRE(tree.at(hit) == c);
    REQUIRE(r == 1);
    REQUIRE(s == 0);
    REQUIRE(t == 0.5f);

    hit = tree.intersect({{0, 0, 0}, {2.5f, 2.5f, 1}}, r, s, t);
    REQUIRE(hit);
    REQUIRE(tree.at(hit) == d);
    REQUIRE(r == 1);
    REQUIRE(s == 0);
    REQUIRE(t == 0.5f);
}


TEST_CASE("KDTree stress test", "[kdtree]")
{
    static constexpr size_t TRIANGLES_COUNT = 100;
    static constexpr size_t RAYS_COUNT = 3400;

    std::cerr << "\n== Stress test ==" << std::endl;

    std::cerr << "Generating " << TRIANGLES_COUNT << " triangles" << std::endl;
    Triangles triangles;

    static std::default_random_engine gen;
    static std::uniform_real_distribution<float> rnd(-0.3f, 0.3f);
    for (size_t i = 0; i < TRIANGLES_COUNT; ++i) {
        Vec center = random_vec();
        Vec v0 = center + (Vec{rnd(gen), rnd(gen), rnd(gen)});
        Vec v1 = center + (Vec{rnd(gen), rnd(gen), rnd(gen)});
        Vec v2 = center + (Vec{rnd(gen), rnd(gen), rnd(gen)});
        Triangle tri = test_triangle(v0, v1, v2);
        triangles.push_back(tri);
    }

    std::cerr << "Generating " << RAYS_COUNT*RAYS_COUNT << " rays."
        << std::endl;
    std::vector<Ray> rays;
    Vec origin{0, 0, 1000};

    float x = -10.f, y = -10.f;
    float step_x = 20.f / 640;
    float step_y = 20.f / 640;

    for (size_t i = 0; i < RAYS_COUNT; ++i, x += step_x) {
        for (size_t j = 0; j < RAYS_COUNT; ++j, y += step_y) {
            rays.emplace_back(origin, (Vec{x, y, 0}) - origin);
        }
    }

    std::cerr << "Building a fast kd-tree of "
        << TRIANGLES_COUNT << " triangles" << std::endl;
    std::cerr << "Node size (bytes): " << KDTree::node_size() << std::endl;
    KDTree tree(triangles);
    std::cerr << "Height  : " << tree.height() << std::endl;
    std::cerr << "Size    : " << tree.size() << std::endl;
    std::cerr << std::endl;

    // test

    std::cerr << "Computing ray fast kd tree intersections" << std::endl;

    float r, s, t;
    size_t hits_fast = 0;
    size_t runtime_ms = 0;

    {
        Runtime runtime(runtime_ms);
        for (const auto& ray : rays) {
            if (tree.intersect(ray, r, s, t)) {
                hits_fast += 1;
            }
        }
    }

    std::cerr << "Runtime  : " << runtime_ms << "ms" << std::endl;
    std::cerr << "# Hits   : " << hits_fast << std::endl;
    std::cerr << "Rays/sec : " << 1000. * RAYS_COUNT / runtime_ms << std::endl;
    std::cerr << std::endl;
}

TEST_CASE("Test cube in kdtree", "[kdtree]") {
    // cube made of triangles
    // front
    auto a1 = test_triangle({-1, -1, -1}, {1, -1, -1}, {1, 1, -1});
    auto a2 = test_triangle({-1, -1, -1}, {-1, 1, -1}, {1, 1, -1});
    // back
    auto a3 = test_triangle({-1, -1, 1}, {1, -1, 1}, {1, 1, 1});
    auto a4 = test_triangle({-1, -1, 1}, {-1, 1, 1}, {1, 1, 1});
    // left
    auto b1 = test_triangle({-1, -1, -1}, {-1, -1, 1}, {-1, 1, 1});
    auto b2 = test_triangle({-1, -1, -1}, {-1, 1, -1}, {-1, 1, 1});
    // right
    auto b3 = test_triangle({1, -1, -1}, {1, -1, 1}, {1, 1, 1});
    auto b4 = test_triangle({1, -1, -1}, {1, 1, -1}, {1, 1, 1});
    // top
    auto c1 = test_triangle({-1, 1, -1}, {-1, 1, 1}, {1, 1, 1});
    auto c2 = test_triangle({-1, 1, -1}, {1, 1, -1}, {1, 1, 1});
    // bottom
    auto c3 = test_triangle({-1, -1, -1}, {-1, -1, 1}, {1, -1, 1});
    auto c4 = test_triangle({-1, -1, -1}, {1, -1, -1}, {1, -1, 1});

    KDTree tree(Triangles{a1, a2, a3, a4, b1, b2, b3, b4, c1, c2, c3, c4});
    REQUIRE(tree.height() == 0);
    REQUIRE(tree.size() == 12);
}

TEST_CASE("All triangles are in the same plane", "[kdtree]")
{
    static std::default_random_engine gen(0);
    static std::uniform_real_distribution<float> rnd(-10.f, 10.f);

    for (Axis ax : AXES) {
        Triangles tris;
        for (size_t i = 0; i < 1000; ++i) {
            tris.push_back(random_triangle_on_unit_sphere(ax, 0));
        }
        KDTree tree(std::move(tris));

        float r, s, t;
        // intersection with parallel ray
        auto ray_pos = Vec{1, 1, 1};
        auto ray_dir = random_vec_on_unit_sphere(ax, 0);

        Ray parallel_ray(ray_pos, ray_dir);
        REQUIRE(!tree.intersect(parallel_ray, r, s, t));

        // intersection with ray through zero
        Ray ray_through_zero({-1, -1, -1}, {1, 1, 1});
        REQUIRE(tree.intersect(ray_through_zero, r, s, t));
    }
}

TEST_CASE("Degenerated triangles test", "[KDTree]") {
    Triangles tris;
    // min 4 triangles to trigger kdtree's bbox splitting
    tris.push_back(test_triangle({0, 0, 0}, {1, 0, 0}, {2, 0, 0}));
    tris.push_back(test_triangle({0, 0, 0}, {1, 0, 0}, {2, 0, 0}));
    tris.push_back(test_triangle({0, 0, 0}, {1, 0, 0}, {2, 0, 0}));
    tris.push_back(test_triangle({0, 0, 0}, {1, 0, 0}, {2, 0, 0}));

    KDTree tree(tris);
    REQUIRE(4 <= tree.size());
}

TEST_CASE("Intersect coplanar triangles", "[KDTree]")
{
    for (auto ax : AXES) {
        Triangles tris;
        for (float pos = 0.f; pos < 10.f; pos += 1.f) {
            tris.push_back(random_regular_triangle_on_unit_sphere(ax, pos));
        }

        KDTree tree(tris);

        Vec origin, dest;
        KDTree::OptionalId triangle_id;
        float r, s, t;

        // ray from negative direction to 0
        origin[ax] = -100;
        dest[ax] = 1;
        triangle_id = tree.intersect(Ray(origin, dest), r, s, t);
        REQUIRE(static_cast<bool>(triangle_id));
        REQUIRE(static_cast<size_t>(triangle_id) == 0L);
        REQUIRE(static_cast<int>(r) == 100);
        REQUIRE(is_eps_zero(s - 1.f/3));
        REQUIRE(is_eps_zero(t - 1.f/3));

        // ray from positive direction to 0
        origin[ax] = 100;
        dest[ax] = -1;
        triangle_id = tree.intersect(Ray(origin, dest), r, s, t);
        REQUIRE(static_cast<bool>(triangle_id));
        REQUIRE(static_cast<size_t>(triangle_id) == 9L);
        REQUIRE(static_cast<int>(r) == 91);
        REQUIRE(is_eps_zero(s - 1.f/3));
        REQUIRE(is_eps_zero(t - 1.f/3));
    }
}

TEST_CASE("Optional id can be stored in unordered containers", "[OptionalId]")
{
    std::unordered_set<KDTree::OptionalId> set;
    set.emplace();
    set.emplace();
    set.emplace(42);
    set.emplace(1);
    set.emplace(42);

    REQUIRE(set.size() == 3);
    REQUIRE(set.count(KDTree::OptionalId()) == 1);
    REQUIRE(set.count(KDTree::OptionalId(1)) == 1);
    REQUIRE(set.count(KDTree::OptionalId(42)) == 1);
}
