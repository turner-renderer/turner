#include "../lib//output.h"
#include "../lib/kdtree.h"
#include "../lib/runtime.h"
#include "helper.h"
#include <catch.hpp>

#include <iostream>
#include <unordered_set>

TEST_CASE("Trivial smoke test", "[kdtree]") {
    auto tri = random_triangle();
    KDTree kdtree({tri});
}

TEST_CASE("Smoke test", "[kdtree]") {
    auto a = test_triangle({-1, -1, 0}, {1, -1, 0}, {1, 1, 0});
    auto b = test_triangle({0, 0, 0}, {1, 0, 0}, {1, 2, 0});
    KDTree tree(Triangles{a, b});
    REQUIRE(tree.height() == 0);
}

TEST_CASE("Four separated triangles test", "[kdtree]") {
    auto a = test_triangle({0, 0, 1}, {0, 1, 1}, {1, 0, 1});
    auto b = test_triangle({2, 0, 1}, {3, 0, 1}, {3, 1, 1});
    auto c = test_triangle({0, 2, 1}, {0, 3, 1}, {1, 3, 1});
    auto d = test_triangle({3, 2, 1}, {3, 3, 1}, {2, 3, 1});
    KDTree tree(Triangles{a, b, c, d});
    REQUIRE(tree.height() == 1);
    REQUIRE(tree.size() == 4);

    KDTreeIntersection tree_intersection(tree);
    KDTree::OptionalId hit;
    float r, s, t;

    hit = tree_intersection.intersect({{0, 0, 0}, {0.5f, 0.5f, 1}}, r, s, t);
    REQUIRE(hit);
    REQUIRE(tree_intersection.at(hit) == a);
    REQUIRE(r == 1);
    REQUIRE(s == 0.5f);
    REQUIRE(t == 0.5f);

    hit = tree_intersection.intersect({{0, 0, 0}, {2.5f, 0.5f, 1}}, r, s, t);
    REQUIRE(hit);
    REQUIRE(tree_intersection.at(hit) == b);
    REQUIRE(r == 1);
    REQUIRE(s == 0);
    REQUIRE(t == 0.5f);

    hit = tree_intersection.intersect({{0, 0, 0}, {0.5f, 2.5f, 1}}, r, s, t);
    REQUIRE(hit);
    REQUIRE(tree_intersection.at(hit) == c);
    REQUIRE(r == 1);
    REQUIRE(s == 0);
    REQUIRE(t == 0.5f);

    hit = tree_intersection.intersect({{0, 0, 0}, {2.5f, 2.5f, 1}}, r, s, t);
    REQUIRE(hit);
    REQUIRE(tree_intersection.at(hit) == d);
    REQUIRE(r == 1);
    REQUIRE(s == 0);
    REQUIRE(t == 0.5f);
}

TEST_CASE("KDTree stress test", "[kdtree]") {
    static constexpr size_t TRIANGLES_COUNT = 100;
    static constexpr size_t RAYS_PER_LINE = 500;

    std::cerr << "\n== Stress test ==" << std::endl;

    std::cerr << "Generating " << TRIANGLES_COUNT << " triangles" << std::endl;
    Triangles triangles;
    triangles.reserve(TRIANGLES_COUNT);
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

    std::cerr << "Building a fast kd-tree of " << TRIANGLES_COUNT
              << " triangles" << std::endl;
    std::cerr << "Node size (bytes): " << KDTree::node_size() << std::endl;
    size_t runtime_ms = 0;
    KDTree tree = ([&] {
        Runtime runtime(runtime_ms);
        return KDTree{std::move(triangles)};
    })();
    std::cerr << "Runtime : " << runtime_ms << "ms" << std::endl;
    std::cerr << "Height  : " << tree.height() << std::endl;
    std::cerr << "Size    : " << tree.size() << std::endl;
    std::cerr << std::endl;

    // test

    std::cerr << "Computing ray fast kd tree intersections" << std::endl;
    size_t num_hits = 0;

    KDTreeIntersection tree_intersection(tree);
    const Vec origin{0, 0, 1000};
    float x = -10.f, y = -10.f;
    const float step_x = 20.f / RAYS_PER_LINE;
    const float step_y = 20.f / RAYS_PER_LINE;
    {
        Runtime runtime(runtime_ms);
        for (size_t i = 0; i < RAYS_PER_LINE; ++i, x += step_x) {
            for (size_t j = 0; j < RAYS_PER_LINE; ++j, y += step_y) {
                float r, s, t;
                Ray ray{origin, (Vec{x, y, 0}) - origin};
                if (tree_intersection.intersect(ray, r, s, t)) {
                    num_hits += 1;
                }
            }
            y = -10.f;
        }
    }

    std::cerr << "Runtime  : " << runtime_ms << "ms" << std::endl;
    std::cerr << "# Rays   : " << RAYS_PER_LINE * RAYS_PER_LINE << std::endl;
    std::cerr << "# Hits   : " << num_hits << std::endl;
    std::cerr << "Rays/sec : " << 1000. * RAYS_PER_LINE / runtime_ms
              << std::endl;
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

TEST_CASE("All triangles are in the same plane", "[kdtree]") {
    for (Axis ax : AXES) {
        Triangles tris;
        for (size_t i = 0; i < 1000; ++i) {
            tris.push_back(random_triangle_on_unit_sphere(ax, 0));
        }
        KDTree tree(std::move(tris));
        KDTreeIntersection tree_intersection(tree);

        float r, s, t;
        // intersection with parallel ray
        auto ray_pos = Vec{1, 1, 1};
        auto ray_dir = random_vec_on_unit_sphere(ax, 0);

        Ray parallel_ray(ray_pos, ray_dir);
        REQUIRE(!tree_intersection.intersect(parallel_ray, r, s, t));

        // intersection with ray through zero
        Ray ray_through_zero({-1, -1, -1}, {1, 1, 1});
        REQUIRE(tree_intersection.intersect(ray_through_zero, r, s, t));
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

TEST_CASE("Intersect coplanar triangles", "[KDTree]") {
    for (auto ax : AXES) {
        Triangles tris;
        for (float pos = 0.f; pos < 10.f; pos += 1.f) {
            tris.push_back(random_regular_triangle_on_unit_sphere(ax, pos));
        }

        KDTree tree(tris);
        KDTreeIntersection tree_intersection(tree);

        Vec origin, dest;
        KDTree::OptionalId triangle_id;
        float r, s, t;

        // ray from negative direction to 0
        origin[ax] = -100;
        dest[ax] = 1;
        triangle_id = tree_intersection.intersect(Ray(origin, dest), r, s, t);
        REQUIRE(static_cast<bool>(triangle_id));
        REQUIRE(static_cast<size_t>(triangle_id) == 0L);
        REQUIRE(static_cast<int>(r) == 100);
        REQUIRE(is_eps_zero(s - 1.f / 3));
        REQUIRE(is_eps_zero(t - 1.f / 3));

        // ray from positive direction to 0
        origin[ax] = 100;
        dest[ax] = -1;
        triangle_id = tree_intersection.intersect(Ray(origin, dest), r, s, t);
        REQUIRE(static_cast<bool>(triangle_id));
        REQUIRE(static_cast<size_t>(triangle_id) == 9L);
        REQUIRE(static_cast<int>(r) == 91);
        REQUIRE(is_eps_zero(s - 1.f / 3));
        REQUIRE(is_eps_zero(t - 1.f / 3));
    }
}

TEST_CASE("Optional id can be stored in unordered containers", "[OptionalId]") {
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

TEST_CASE("Inner FlatNode is constructed correctly", "[FlatNode]") {
    float split_pos = 3.1415;
    uint32_t right_index = 42;
    {
        detail::FlatNode node(Axis::X, split_pos, right_index);
        REQUIRE(node.is_inner());
        REQUIRE(!node.is_leaf());
        REQUIRE(node.split_axis() == Axis::X);
        REQUIRE(node.split_pos() == split_pos);
        REQUIRE(node.right() == right_index);
    }
    {
        detail::FlatNode node(Axis::Y, split_pos, right_index);
        REQUIRE(node.is_inner());
        REQUIRE(!node.is_leaf());
        REQUIRE(node.split_axis() == Axis::Y);
        REQUIRE(node.split_pos() == split_pos);
        REQUIRE(node.right() == right_index);
    }
    {
        detail::FlatNode node(Axis::Z, split_pos, right_index);
        REQUIRE(node.is_inner());
        REQUIRE(!node.is_leaf());
        REQUIRE(node.split_axis() == Axis::Z);
        REQUIRE(node.split_pos() == split_pos);
        REQUIRE(node.right() == right_index);
    }
}

TEST_CASE("Set right of inner FlatNode", "[FlatNode]") {
    float split_pos = 3.1415;
    {
        detail::FlatNode node(Axis::X, split_pos, 0);
        node.set_right(42);
        REQUIRE(node.right() == 42);
    }
    {
        detail::FlatNode node(Axis::Y, split_pos, 0);
        node.set_right(42);
        REQUIRE(node.right() == 42);
    }
    {
        detail::FlatNode node(Axis::Z, split_pos, 0);
        node.set_right(42);
        REQUIRE(node.right() == 42);
    }
}

TEST_CASE("Leaf FlatNode is constructed correctly", "[FlatNode]") {
    {
        detail::FlatNode node;
        REQUIRE(!node.is_inner());
        REQUIRE(node.is_leaf());
        REQUIRE(node.is_empty());
        REQUIRE(node.is_sentinel());
    }
    {
        detail::FlatNode node(1);
        REQUIRE(!node.is_inner());
        REQUIRE(node.is_leaf());
        REQUIRE(!node.is_empty());
        REQUIRE(node.is_sentinel());
        REQUIRE(node.first_triangle_id() == 1);
    }
    {
        detail::FlatNode node(1, 2);
        REQUIRE(!node.is_inner());
        REQUIRE(node.is_leaf());
        REQUIRE(!node.is_empty());
        REQUIRE(!node.is_sentinel());
        REQUIRE(node.first_triangle_id() == 1);
        REQUIRE(node.second_triangle_id() == 2);
    }
}
