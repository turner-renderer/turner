#include "helper.h"
#include "../lib/kdtree.h"
#include "../lib//output.h"
#include "../lib/runtime.h"
#include <catch.hpp>

#include <iostream>


// Extend the interface of FastKDTree for testing.
class FastKDTreeTester {
public:
    FastKDTreeTester(FastKDTree& tree) : tree(tree) {}

    static size_t height(std::unique_ptr<FastKDTree::Node>& ptr) {
        if (!ptr) {
            return 0;
        }
        if (ptr->type == FastKDTree::NodeType::LEAF) {
            return 1;
        }

        FastKDTree::InnerNode& node = *static_cast<FastKDTree::InnerNode*>(ptr.get());

        return 1 + std::max(height(node.lft), height(node.rht));
    }

    size_t height() {
        return height(tree.root_);
    }

    static size_t size(std::unique_ptr<FastKDTree::Node>& ptr) {
        if (!ptr) {
            return 0;
        }
        if (ptr->type == FastKDTree::NodeType::LEAF) {
            return static_cast<FastKDTree::Leaf*>(ptr.get())->tris.size();
        }

        FastKDTree::InnerNode& node = *static_cast<FastKDTree::InnerNode*>(ptr.get());

        return size(node.lft) + size(node.rht);
    }

    size_t size() {
        return size(tree.root_);
    }

    bool is_leaf() {
        return tree.root_ && tree.root_->type == FastKDTree::NodeType::LEAF;
    }

    Triangles& tris() {
        auto node = static_cast<FastKDTree::Leaf*>(tree.root_.get());
        return node->tris;
    }

    FastKDTree& tree;
};


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
    FastKDTreeTester tester(tree);

    REQUIRE(tester.height() == 1);
    REQUIRE(tester.is_leaf());
}

TEST_CASE("KDTree stress test", "[kdtree]")
{
    static constexpr size_t TRIANGLES_COUNT = 1000;

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

    size_t runtime_ms;
    {
        Runtime runtime(runtime_ms);
        FastKDTree tree(triangles);
        FastKDTreeTester tree_tester(tree);
        std::cerr << "Height  : " << tree_tester.height() << std::endl;
        std::cerr << "Size    : " << tree_tester.size() << std::endl;
    }
    std::cerr << "Runtime : " << runtime_ms << "ms" << std::endl;

}
