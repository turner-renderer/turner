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

    Incidence inc;
    inc.add_triangle(42, tri);
    inc.add_subdivision(42, tri, {0, 1, 2, 3}, children);

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

    Incidence inc;
    inc.add_triangle(42, tri);
    inc.add_subdivision(42, tri, {0, 1, 2, 3}, children_lvl1);

    const Vec ma = {2, 2, 0};
    const Vec mb = {-2, 2, 0};
    const Vec mc = {0, 0, 0};

    const auto children_lvl2 = subdivide4(children_lvl1[1]);
    inc.add_subdivision(1, children_lvl1[1], {4, 5, 6, 7}, children_lvl2);

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

    REQUIRE(inc.side_incidence().size() == 15);
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
