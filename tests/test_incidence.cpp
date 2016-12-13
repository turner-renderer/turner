#include "../lib/hierarchical.h"
#include "../lib/incidence.h"
#include "helper.h"

#include <catch.hpp>

using TriangleId = KDTree::TriangleId;
using TriangleIds = std::unordered_set<TriangleId>;

TEST_CASE("Add triangle", "[incidence]") {
    TriangleId tri_id = 42;

    const Vec a = {-2, 0, 0};
    const Vec b = {2, 0, 0};
    const Vec c = {0, 2, 0};

    const Vec n = {0, 0, 1};

    const auto tri = test_triangle(a, b, c, n, n, n);

    Incidence inc;
    inc.add_triangle(tri_id, tri);

    REQUIRE(inc.vertex_incidence().size() == 3);
    REQUIRE(inc.vertex_incidence({a, n}) == (TriangleIds{tri_id}));
    REQUIRE(inc.vertex_incidence({b, n}) == (TriangleIds{tri_id}));
    REQUIRE(inc.vertex_incidence({c, n}) == (TriangleIds{tri_id}));

    REQUIRE(inc.side_incidence().size() == 3);
    REQUIRE(inc.side_incidence({{a, n}, {b, n}}) == (TriangleIds{tri_id}));
    REQUIRE(inc.side_incidence({{b, n}, {c, n}}) == (TriangleIds{tri_id}));
    REQUIRE(inc.side_incidence({{c, n}, {a, n}}) == (TriangleIds{tri_id}));
}

TEST_CASE("Top-level triangle incidence", "[incidence]") {
    const Vec a = {-2, 0, 0};
    const Vec b = {2, 0, 0};
    const Vec c = {0, 2, 0};

    const Vec ma = {1, 1, 0};
    const Vec mb = {-1, 1, 0};
    const Vec mc = {0, 0, 0};

    const Vec n = {0, 0, 1};

    const auto tri = test_triangle(a, b, c, n, n, n);
    const auto children = subdivide4(tri);
    std::array<const Triangle*, 4> children_ptrs = {&children[0], &children[1],
                                                    &children[2], &children[3]};

    Incidence inc;
    inc.add_triangle(42, tri);
    inc.add_subdivision(42, tri, {0, 1, 2, 3}, children_ptrs);

    REQUIRE(inc.vertex_incidence().size() == 6);
    REQUIRE(inc.vertex_incidence({a, n}) == (TriangleIds{0}));
    REQUIRE(inc.vertex_incidence({mc, n}) == (TriangleIds{0, 1, 3}));
    REQUIRE(inc.vertex_incidence({b, n}) == (TriangleIds{1}));
    REQUIRE(inc.vertex_incidence({ma, n}) == (TriangleIds{1, 2, 3}));
    REQUIRE(inc.vertex_incidence({c, n}) == (TriangleIds{2}));
    REQUIRE(inc.vertex_incidence({mb, n}) == (TriangleIds{0, 2, 3}));

    REQUIRE(inc.side_incidence().size() == 9);
    REQUIRE(inc.side_incidence({{a, n}, {mc, n}}) == (TriangleIds{0}));
    REQUIRE(inc.side_incidence({{mc, n}, {b, n}}) == (TriangleIds{1}));
    REQUIRE(inc.side_incidence({{b, n}, {ma, n}}) == (TriangleIds{1}));
    REQUIRE(inc.side_incidence({{ma, n}, {c, n}}) == (TriangleIds{2}));
    REQUIRE(inc.side_incidence({{c, n}, {mb, n}}) == (TriangleIds{2}));
    REQUIRE(inc.side_incidence({{mb, n}, {a, n}}) == (TriangleIds{0}));
    REQUIRE(inc.side_incidence({{ma, n}, {mb, n}}) == (TriangleIds{2, 3}));
    REQUIRE(inc.side_incidence({{mb, n}, {mc, n}}) == (TriangleIds{0, 3}));
    REQUIRE(inc.side_incidence({{mc, n}, {ma, n}}) == (TriangleIds{1, 3}));
}

TEST_CASE("Inner triangle incidence", "[incidence]") {
    const Vec a = {-4, 0, 0};
    const Vec b = {4, 0, 0};
    const Vec c = {0, 4, 0};

    const Vec n = {0, 0, 1};

    const auto tri = test_triangle(a, b, c, n, n, n);
    const auto children_lvl1 = subdivide4(tri);
    std::array<const Triangle*, 4> children_lvl1_ptrs = {
        &children_lvl1[0], &children_lvl1[1], &children_lvl1[2],
        &children_lvl1[3]};

    Incidence inc;
    inc.add_triangle(42, tri);
    inc.add_subdivision(42, tri, {0, 1, 2, 3}, children_lvl1_ptrs);

    const Vec ma = {2, 2, 0};
    const Vec mb = {-2, 2, 0};
    const Vec mc = {0, 0, 0};

    const auto children_lvl2 = subdivide4(children_lvl1[1]);
    std::array<const Triangle*, 4> children_lvl2_ptrs = {
        &children_lvl2[0], &children_lvl2[1], &children_lvl2[2],
        &children_lvl2[3]};
    inc.add_subdivision(1, children_lvl1[1], {4, 5, 6, 7}, children_lvl2_ptrs);

    const Vec mma = {3, 1, 0};
    const Vec mmb = {1, 1, 0};
    const Vec mmc = {2, 0, 0};

    REQUIRE(inc.vertex_incidence().size() == 9);
    REQUIRE(inc.vertex_incidence({a, n}) == (TriangleIds{0}));
    REQUIRE(inc.vertex_incidence({mc, n}) == (TriangleIds{0, 3, 4}));
    REQUIRE(inc.vertex_incidence({mmc, n}) == (TriangleIds{4, 5, 7}));
    REQUIRE(inc.vertex_incidence({b, n}) == (TriangleIds{5}));
    REQUIRE(inc.vertex_incidence({mmb, n}) == (TriangleIds{3, 4, 6, 7}));
    REQUIRE(inc.vertex_incidence({mma, n}) == (TriangleIds{5, 6, 7}));
    REQUIRE(inc.vertex_incidence({mb, n}) == (TriangleIds{0, 3, 2}));
    REQUIRE(inc.vertex_incidence({ma, n}) == (TriangleIds{2, 3, 6}));
    REQUIRE(inc.vertex_incidence({c, n}) == (TriangleIds{2}));

    REQUIRE(inc.side_incidence().size() == 16);
    REQUIRE(inc.side_incidence({{a, n}, {mc, n}}) == (TriangleIds{0}));
    REQUIRE(inc.side_incidence({{mc, n}, {mmc, n}}) == (TriangleIds{4}));
    REQUIRE(inc.side_incidence({{mmc, n}, {b, n}}) == (TriangleIds{5}));
    REQUIRE(inc.side_incidence({{b, n}, {mma, n}}) == (TriangleIds{5}));
    REQUIRE(inc.side_incidence({{mma, n}, {ma, n}}) == (TriangleIds{6}));
    REQUIRE(inc.side_incidence({{ma, n}, {c, n}}) == (TriangleIds{2}));
    REQUIRE(inc.side_incidence({{c, n}, {mb, n}}) == (TriangleIds{2}));
    REQUIRE(inc.side_incidence({{mb, n}, {a, n}}) == (TriangleIds{0}));
    REQUIRE(inc.side_incidence({{mc, n}, {mmb, n}}) == (TriangleIds{3, 4}));
    REQUIRE(inc.side_incidence({{mmb, n}, {ma, n}}) == (TriangleIds{3, 6}));
    REQUIRE(inc.side_incidence({{ma, n}, {mb, n}}) == (TriangleIds{2, 3}));
    REQUIRE(inc.side_incidence({{ma, n}, {mb, n}}) == (TriangleIds{2, 3}));
    REQUIRE(inc.side_incidence({{mmc, n}, {mma, n}}) == (TriangleIds{5, 7}));
    REQUIRE(inc.side_incidence({{mma, n}, {mmb, n}}) == (TriangleIds{6, 7}));
    REQUIRE(inc.side_incidence({{mmb, n}, {mmc, n}}) == (TriangleIds{4, 7}));
}

std::array<const Triangle*, 4>
get_children_ptrs(const std::array<Triangle, 4>& children) {
    return {&children[0], &children[1], &children[2], &children[3]};
}

TEST_CASE("Full subdivision inner triangle incidence", "[incidence]") {
    const Vec a = {-4, 0, 0};
    const Vec b = {4, 0, 0};
    const Vec c = {0, 4, 0};

    const Vec n = {0, 0, 1};

    const auto tri = test_triangle(a, b, c, n, n, n);
    const auto children_lvl1 = subdivide4(tri);
    const auto children_lvl1_ptrs = get_children_ptrs(children_lvl1);

    Incidence inc;
    inc.add_triangle(42, tri);
    inc.add_subdivision(42, tri, {0, 1, 2, 3}, children_lvl1_ptrs);

    {
        const auto children_lvl2 = subdivide4(children_lvl1[0]);
        const auto children_lvl2_ptrs = get_children_ptrs(children_lvl2);
        inc.add_subdivision(0, children_lvl1[0], {4, 5, 6, 7},
                            children_lvl2_ptrs);
    }
    {
        const auto children_lvl2 = subdivide4(children_lvl1[1]);
        const auto children_lvl2_ptrs = get_children_ptrs(children_lvl2);
        inc.add_subdivision(1, children_lvl1[1], {8, 9, 10, 11},
                            children_lvl2_ptrs);
    }
    {
        const auto children_lvl2 = subdivide4(children_lvl1[2]);
        const auto children_lvl2_ptrs = get_children_ptrs(children_lvl2);
        inc.add_subdivision(2, children_lvl1[2], {12, 13, 14, 15},
                            children_lvl2_ptrs);
    }
    {
        const auto children_lvl2 = subdivide4(children_lvl1[3]);
        const auto children_lvl2_ptrs = get_children_ptrs(children_lvl2);
        inc.add_subdivision(3, children_lvl1[3], {16, 17, 18, 19},
                            children_lvl2_ptrs);
    }

    REQUIRE(inc.vertex_incidence({{-4, 0, 0}, n}) == (TriangleIds{4}));
    REQUIRE(inc.vertex_incidence({{-2, 0, 0}, n}) == (TriangleIds{4, 5, 7}));
    REQUIRE(inc.vertex_incidence({{0, 0, 0}, n}) == (TriangleIds{5, 8, 17}));
    REQUIRE(inc.vertex_incidence({{2, 0, 0}, n}) == (TriangleIds{8, 9, 11}));
    REQUIRE(inc.vertex_incidence({{4, 0, 0}, n}) == (TriangleIds{9}));

    REQUIRE(inc.vertex_incidence({{-3, 1, 0}, n}) == (TriangleIds{4, 6, 7}));
    REQUIRE(inc.vertex_incidence({{-1, 1, 0}, n}) ==
            (TriangleIds{5, 6, 7, 16, 17, 19}));
    REQUIRE(inc.vertex_incidence({{1, 1, 0}, n}) ==
            (TriangleIds{8, 10, 11, 17, 18, 19}));
    REQUIRE(inc.vertex_incidence({{3, 1, 0}, n}) == (TriangleIds{9, 10, 11}));

    REQUIRE(inc.vertex_incidence({{-2, 2, 0}, n}) == (TriangleIds{6, 12, 16}));
    REQUIRE(inc.vertex_incidence({{0, 2, 0}, n}) ==
            (TriangleIds{12, 13, 15, 16, 18, 19}));
    REQUIRE(inc.vertex_incidence({{2, 2, 0}, n}) == (TriangleIds{10, 13, 18}));

    REQUIRE(inc.vertex_incidence({{-1, 3, 0}, n}) == (TriangleIds{12, 14, 15}));
    REQUIRE(inc.vertex_incidence({{1, 3, 0}, n}) == (TriangleIds{13, 14, 15}));

    REQUIRE(inc.vertex_incidence({{0, 4, 0}, n}) == (TriangleIds{14}));
}
