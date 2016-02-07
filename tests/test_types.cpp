#include "../lib/types.h"
#include "../lib/triangle.h"
#include "../lib/kdtree.h"
#include "../lib/output.h"
#include <catch.hpp>
#include <random>
#include <iostream>


// We need this operator only for tests.
bool operator==(const Box& b1, const Box& b2) {
    return b1.min == b2.min && b2.max == b2.max;
}


// Construct a triangle with trivial normals and colors.
Triangle test_triangle(Vec a, Vec b, Vec c) {
    return Triangle({{a, b, c}}, {{Vec{}, Vec{}, Vec{}}}, {}, {});
}


// We need this operator only for tests.
bool operator==(const Triangle& tria, const Triangle& trib) {
    return
        tria.vertices[0] == trib.vertices[0] &&
        tria.vertices[1] == trib.vertices[1] &&
        tria.vertices[2] == trib.vertices[2];
}


TEST_CASE("Test Box expand", "[box]")
{
    Box box{{0, 0, 0}, {1, 1, 1}};
    REQUIRE(box.expand({{0, 0, 0}, {0, 0, 0}}) == box);
    REQUIRE(box.expand({{0, 0, 0}, {1, 1, 1}}) == box);
    REQUIRE(box.expand({{0, 0, 0}, {0.5f, 0.5f, 0.5f}}) == box);
    REQUIRE(box.expand({{0, 0, 0}, {2, 1, 1}}) == (Box{{0, 0, 0}, {2, 1, 1}}));
    REQUIRE(box.expand({{0, 0, 0}, {1, 2, 1}}) == (Box{{0, 0, 0}, {1, 2, 1}}));
    REQUIRE(box.expand({{0, 0, 0}, {1, 1, 2}}) == (Box{{0, 0, 0}, {1, 1, 2}}));
    REQUIRE(box.expand({{-1, 0, 0},{1, 1, 1}}) == (Box{{-1, 0, 0},{0, 0, 0}}));
    REQUIRE(box.expand({{0, -1, 0},{1, 1, 1}}) == (Box{{0, -1, 0},{0, 0, 0}}));
    REQUIRE(box.expand({{0, 0, -1},{1, 1, 1}}) == (Box{{0, 0, -1},{0, 0, 0}}));
}


TEST_CASE("Test bbox of triangle", "[triangle]")
{
    REQUIRE(
        test_triangle({0, 0, 0}, {0, 0, 1}, {0, 1, 1}).bbox()
        == (Box{{0, 0, 0}, {0, 1, 1}}));
    REQUIRE(
        test_triangle({-1, 0, 0}, {0, 0, 1}, {0, 1, 1}).bbox()
        == (Box{{-1, 0, 0}, {0, 1, 1}}));
    REQUIRE(
        test_triangle({0, 0, 0}, {0, -1, 1}, {0, 1, 1}).bbox()
        == (Box{{0, -1, 0}, {0, 1, 1}}));

    REQUIRE(
        test_triangle({0, 0, 0}, {0, 0, -1}, {0, 1, 1}).bbox()
        == (Box{{0, 0, -1}, {0, 1, 1}}));

    REQUIRE(
        test_triangle({0, 0, 0}, {0, 0, 1}, {2, 1, 1}).bbox()
        == (Box{{0, 0, 0}, {2, 1, 1}}));
}


// Extend the interface of KDTree for testing.
template<unsigned int LEAF_CAPACITY>
class KDTreeTester {
public:
    using KDTreeT = KDTree<LEAF_CAPACITY>;
    using KDTreeNode = typename KDTreeT::Node;

    KDTreeTester(const KDTreeT& tree) : tree(tree) {}
    size_t bytes() const {
        if (tree.empty()) {
            return 0;
        }

        const KDTreeNode& node = *tree.root_.get();
        return sizeof(node.bbox)
            + sizeof(node.splitaxis)
            + sizeof(Triangle) * node.triangles.size()
            + KDTreeTester<LEAF_CAPACITY>(tree.left()).bytes()
            + KDTreeTester<LEAF_CAPACITY>(tree.right()).bytes();
    }

    const KDTreeT& tree;
};


TEST_CASE("KDTree smoke test", "[kdtree]")
{
    auto a = test_triangle({-1, -1, 0}, {1, -1, 0}, {1, 1, 0});
    auto b = test_triangle({0, 0, 0}, {1, 0, 0}, {1, 2, 0});
    KDTree<10> tree10(Triangles{a, b});

    REQUIRE(tree10.height() == 1);
    REQUIRE(tree10.is_leaf());

    KDTree<1> tree1(Triangles{a, b});

    REQUIRE(tree1.height() == 2);
    REQUIRE(!tree1.is_leaf());
    REQUIRE(tree1.left().is_leaf());
    REQUIRE(tree1.right().is_leaf());

    REQUIRE(tree1.left().triangles().size() == 1);
    REQUIRE(tree1.left().triangles().at(0) == a);
    REQUIRE(tree1.left().bbox() == a.bbox());

    REQUIRE(tree1.right().triangles().size() == 1);
    REQUIRE(tree1.right().triangles().at(0) == b);
    REQUIRE(tree1.right().bbox() == b.bbox());
}

TEST_CASE("KDTree stress test", "[kdtree]")
{
    static constexpr size_t COUNT = 2000000;

    std::cerr << "Generating triangles" << std::endl;
    Triangles triangles;
    std::default_random_engine gen;
    std::uniform_real_distribution<float> rand(-100.f, 100.f);
    for (size_t i = 0; i < COUNT; ++i) {
        triangles.push_back(test_triangle(
            {rand(gen), rand(gen), rand(gen)},
            {rand(gen), rand(gen), rand(gen)},
            {rand(gen), rand(gen), rand(gen)}));
    }

    std::cerr << "Building the kd-tree" << std::endl;
    KDTree<10> tree(triangles);
    KDTreeTester<10> tree_tester(tree);

    auto triangles_bytes = triangles.size() * sizeof(Triangle);
    auto tree_bytes = tree_tester.bytes();
    std::cerr << "# Triangles       : " << COUNT << std::endl;
    std::cerr << "Size of triangles : "
        << triangles_bytes / 1024 / 1024 << "MB" << std::endl;
    std::cerr << "Size of kd-tree   : "
        << tree_bytes / 1024 / 1024 << "MB" << std::endl;
    std::cerr << "Overhead          : "
        << (100. * tree_bytes / triangles_bytes - 100.) << "%" << std::endl;

    REQUIRE((tree.height() == 18 || tree.height() == 19));
}
