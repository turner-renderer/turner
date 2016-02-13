#include "helper.h"
#include "../lib/kdtree.h"
#include <catch.hpp>

#include <iostream>


TEST_CASE("Smoke test", "[kdtree]")
{
    auto tri = random_triangle();
    FastKDTree kdtree({tri});
}


TEST_CASE("KDTree stress test", "[kdtree]")
{
    static constexpr size_t TRIANGLES_COUNT = 1000000;
    static constexpr unsigned int LEAF_CAPACITY = 100;

    std::cerr << "\n== Stress test ==" << std::endl;

    std::cerr << "Generating " << TRIANGLES_COUNT << " triangles" << std::endl;
    Triangles triangles;
    for (size_t i = 0; i < TRIANGLES_COUNT; ++i) {
        triangles.push_back(random_triangle());
    }

    std::cerr << "Building a kd-tree of " << TRIANGLES_COUNT << " triangles"
        << std::endl;



    FastKDTree tree(triangles);
    // KDTreeTester tree_tester(tree);
    // auto triangles_bytes = triangles.size() * sizeof(Triangle);
    // auto tree_bytes = tree_tester.bytes();
    // std::cerr << "Leaf capacity     : " << LEAF_CAPACITY << std::endl;
    // std::cerr << "Size of triangles : "
    //     << triangles_bytes / 1024 / 1024 << "MB" << std::endl;
    // std::cerr << "Size of kd-tree   : "
    //     << tree_bytes / 1024 / 1024 << "MB" << std::endl;
    // std::cerr << "Overhead          : "
    //     << (100. * tree_bytes / triangles_bytes - 100.) << "%" << std::endl;
    // std::cerr << "Height            : " << tree.height() << std::endl;
}
