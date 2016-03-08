#include "helper.h"
#include "../lib/kdtree.h"
#include "../lib//output.h"
#include "../lib/runtime.h"
#include <catch.hpp>

#include <iostream>


TEST_CASE("Split box at plane", "[box]") {
    Box box{{0, 0, 0}, {2, 2, 2}};

    Box l, r;
    std::tie(l, r) = split(box, Plane{Axis::X, 1});
    REQUIRE(l == (Box{{0, 0, 0}, {1, 2, 2}}));
    REQUIRE(r == (Box{{1, 0, 0}, {2, 2, 2}}));

    std::tie(l, r) = split(box, Plane{Axis::Y, 1});
    REQUIRE(l == (Box{{0, 0, 0}, {2, 1, 2}}));
    REQUIRE(r == (Box{{0, 1, 0}, {2, 2, 2}}));

    std::tie(l, r) = split(box, Plane{Axis::Z, 1});
    REQUIRE(l == (Box{{0, 0, 0}, {2, 2, 1}}));
    REQUIRE(r == (Box{{0, 0, 1}, {2, 2, 2}}));
}


TEST_CASE("Trivial smoke test", "[kdtree]")
{
    auto tri = random_triangle();
    FastKDTree kdtree({tri});
}

TEST_CASE("Smoke test", "[kdtree]")
{
    auto a = test_triangle({-1, -1, 0}, {1, -1, 0}, {1, 1, 0});
    auto b = test_triangle({0, 0, 0}, {1, 0, 0}, {1, 2, 0});
    FastKDTree tree(Triangles{a, b});
    REQUIRE(tree.height() == 0);
}

TEST_CASE("Four separated triangle test", "[kdtree]")
{
    auto a = test_triangle({0, 0, 0}, {0, 1, 0}, {1, 0, 0});
    auto b = test_triangle({2, 0, 0}, {3, 0, 0}, {3, 1, 0});
    auto c = test_triangle({0, 2, 0}, {0, 3, 0}, {1, 3, 0});
    auto d = test_triangle({3, 2, 0}, {3, 3, 0}, {2, 3, 0});
    FastKDTree tree(Triangles{a, b, c, d});
    REQUIRE(tree.height() == 3);
    REQUIRE(tree.size() == 4);
}

TEST_CASE("KDTree stress test", "[kdtree]")
{
    static constexpr size_t TRIANGLES_COUNT = 100;

    std::cerr << "\n== Stress test ==" << std::endl;

    std::cerr << "Generating " << TRIANGLES_COUNT << " triangles" << std::endl;
    Triangles triangles;

    static std::default_random_engine gen;
    static std::uniform_real_distribution<float> rnd(-0.5f, 0.5f);
    for (size_t i = 0; i < TRIANGLES_COUNT; ++i) {
        Vec center = random_vec();
        Vec v0 = center + (Vec{rnd(gen), rnd(gen), rnd(gen)});
        Vec v1 = center + (Vec{rnd(gen), rnd(gen), rnd(gen)});
        Vec v2 = center + (Vec{rnd(gen), rnd(gen), rnd(gen)});
        Triangle tri = test_triangle(v0, v1, v2);

        triangles.push_back(tri);
    }

    std::cerr << "Building a kd-tree of " << TRIANGLES_COUNT << " triangles"
        << std::endl;
    std::cerr << "Node size (bytes): " << FastKDTree::node_size() << std::endl;

    size_t runtime_ms;
    {
        Runtime runtime(runtime_ms);
        FastKDTree tree(triangles);
        std::cerr << "Height  : " << tree.height() << std::endl;
        std::cerr << "Size    : " << tree.size() << std::endl;
    }
    std::cerr << "Runtime : " << runtime_ms << "ms" << std::endl;
}
