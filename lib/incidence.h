#pragma once

#include "functional.h"
#include "kdtree.h"

#include <array>
#include <unordered_map>
#include <unordered_set>

using TriangleVertex = std::pair<Vec /* vertex */, Vec /* normal */>;

/**
 * Triangle side is uniquely identified by two unordered triangle vertices.
 */
class TriangleSide {
public:
    TriangleSide(const TriangleVertex& a, const TriangleVertex& b)
        : data_(std::minmax(a, b, vertex_cmp)) {}

    constexpr const TriangleVertex& a() const { return data_.first; }
    constexpr const TriangleVertex& b() const { return data_.second; }

    constexpr bool operator==(const TriangleSide& other) const {
        return data_ == other.data_;
    }

private:
    /** Compare two vectors lexicographically. */
    static bool lexic_vec_cmp(const Vec& a, const Vec& b) {
        return a.x < b.x
                   ? true
                   : a.x > b.x
                         ? false
                         : a.y < b.y ? true : a.y > b.y ? false : a.z < b.z;
    }

    /** Compare two triangle vertices lexicographically. */
    static bool vertex_cmp(const TriangleVertex& a, const TriangleVertex& b) {
        return lexic_vec_cmp(a.first, b.first)
                   ? true
                   : lexic_vec_cmp(b.first, a.first)
                         ? false
                         : lexic_vec_cmp(a.second, b.second);
    }

private:
    std::pair<TriangleVertex, TriangleVertex> data_;
};

/** Triangle side hash specialization */
namespace std {
template <> struct hash<TriangleSide> {
    size_t operator()(const TriangleSide& side) const {
        size_t seed = 0;
        hash_combine(seed, side.a(), side.b());
        return seed;
    }
};
} // namespace std

class Incidence {
private:
    using TriangleId = KDTree::TriangleId;
    using TriangleIds = std::unordered_set<TriangleId>;

public:
    bool empty() const {
        return vertex_incidence_.empty() && side_incidence_.empty();
    }

    const auto& vertex_incidence() const { return vertex_incidence_; }

    const TriangleIds& vertex_incidence(const TriangleVertex& v) const {
        return vertex_incidence_.at(v);
    }

    const TriangleIds& side_incidence(const TriangleSide& side) const {
        return side_incidence_.at(side);
    }

    const auto& side_incidence() const { return side_incidence_; }

    void add_triangle(TriangleId tri_id, const Triangle& tri);

    /**
     * @note Assumption (!): Subdivision was done by method `subdivide4`.
     */
    void add_subdivision(TriangleId tri_id, const Triangle& tri,
                         const std::array<TriangleId, 4>& children_ids,
                         const std::array<const Triangle*, 4>& children);

    size_t vertex_index(TriangleId tri_id, const TriangleVertex& vertex) const {
        return vertex_index_.at({tri_id, vertex});
    }

private:
    /** Get triangle vertex by index */
    TriangleVertex vertex(const Triangle& tri, size_t i) const {
        return {tri.vertices[i], tri.normals[i]};
    };

    /** Get triangle side by index */
    TriangleSide side(const Triangle& tri, size_t i) {
        return {vertex(tri, i), vertex(tri, (i + 1) % 3)};
    };

    void add_side_incidence(const TriangleSide& side, TriangleId tri_id) {
        side_incidence_[side].insert(tri_id);
    }

    /** Compare two vectors lexicographically. */
    static bool lexic_vec_cmp(const Vec& a, const Vec& b) {
        return a.x < b.x
                   ? true
                   : a.x > b.x
                         ? false
                         : a.y < b.y ? true : a.y > b.y ? false : a.z < b.z;
    }

    /** Compare two triangle vertices lexicographically. */
    static bool vertex_cmp(const TriangleVertex& a, const TriangleVertex& b) {
        return lexic_vec_cmp(a.first, b.first)
                   ? true
                   : lexic_vec_cmp(b.first, a.first)
                         ? false
                         : lexic_vec_cmp(a.second, b.second);
    }

private:
    std::unordered_map<TriangleVertex, TriangleIds> vertex_incidence_;
    std::unordered_map<TriangleSide, TriangleIds> side_incidence_;
    std::unordered_map<std::pair<TriangleId, TriangleVertex>,
                       size_t /* index */>
        vertex_index_;
};

// -----------------------------------------------------------------------------

inline void Incidence::add_triangle(TriangleId tri_id, const Triangle& tri) {
    for (size_t i = 0; i < 3; ++i) {
        const auto& v = vertex(tri, i);
        vertex_incidence_[v].insert(tri_id);
        vertex_index_[{tri_id, v}] = i;
        side_incidence_[side(tri, i)].insert(tri_id);
    }
}

inline void
Incidence::add_subdivision(TriangleId tri_id, const Triangle& tri,
                           const std::array<TriangleId, 4>& children_ids,
                           const std::array<const Triangle*, 4>& children) {
    auto a = vertex(tri, 0);
    auto b = vertex(tri, 1);
    auto c = vertex(tri, 2);

    auto mc = vertex(*children[0], 1);
    auto ma = vertex(*children[1], 2);
    auto mb = vertex(*children[2], 0);

    auto side_c = side(tri, 0); // ab
    auto side_a = side(tri, 1); // bc
    auto side_b = side(tri, 2); // ca

    // erase vertex incidence of current triangle
    {
        auto it = vertex_incidence_[a].find(tri_id);
        assert(it != vertex_incidence_[a].end());
        vertex_incidence_[a].erase(it);
    }
    {
        auto it = vertex_incidence_[b].find(tri_id);
        assert(it != vertex_incidence_[b].end());
        vertex_incidence_[b].erase(it);
    }
    {
        auto it = vertex_incidence_[c].find(tri_id);
        assert(it != vertex_incidence_[c].end());
        vertex_incidence_[c].erase(it);
    }
    {
        auto it = vertex_incidence_[ma].find(tri_id);
        if (it != vertex_incidence_[ma].end()) {
            vertex_incidence_[ma].erase(it);
        }
    }
    {
        auto it = vertex_incidence_[mb].find(tri_id);
        if (it != vertex_incidence_[mb].end()) {
            vertex_incidence_[mb].erase(it);
        }
    }
    {
        auto it = vertex_incidence_[mc].find(tri_id);
        if (it != vertex_incidence_[mc].end()) {
            vertex_incidence_[mc].erase(it);
        }
    }

    // erase side incidence of current triangle
    {
        auto it = side_incidence_[side_c].find(tri_id);
        assert(it != side_incidence_[side_c].end());
        side_incidence_[side_c].erase(it);
    }
    {
        auto it = side_incidence_[side_a].find(tri_id);
        assert(it != side_incidence_[side_a].end());
        side_incidence_[side_a].erase(it);
    }
    {
        auto it = side_incidence_[side_b].find(tri_id);
        assert(it != side_incidence_[side_b].end());
        side_incidence_[side_b].erase(it);
    }

    // propagate side incidences from parent to children
    for (auto tri_id : side_incidence_[side_c]) {
        side_incidence_[{a, mc}].insert(tri_id);
        side_incidence_[{mc, b}].insert(tri_id);
    }
    for (auto tri_id : side_incidence_[side_a]) {
        side_incidence_[{b, ma}].insert(tri_id);
        side_incidence_[{ma, c}].insert(tri_id);
    }
    for (auto tri_id : side_incidence_[side_b]) {
        side_incidence_[{c, mb}].insert(tri_id);
        side_incidence_[{mb, a}].insert(tri_id);
    }

    // add children to side incidences
    side_incidence_[{a, mc}].insert(children_ids[0]);
    side_incidence_[{mc, b}].insert(children_ids[1]);
    side_incidence_[{b, ma}].insert(children_ids[1]);
    side_incidence_[{ma, c}].insert(children_ids[2]);
    side_incidence_[{c, mb}].insert(children_ids[2]);
    side_incidence_[{mb, a}].insert(children_ids[0]);
    side_incidence_[{mc, ma}].insert(children_ids[1]);
    side_incidence_[{mc, ma}].insert(children_ids[3]);
    side_incidence_[{ma, mb}].insert(children_ids[2]);
    side_incidence_[{ma, mb}].insert(children_ids[3]);
    side_incidence_[{mb, mc}].insert(children_ids[0]);
    side_incidence_[{mb, mc}].insert(children_ids[3]);

    // add vertex incidences
    for (size_t i = 0; i < 4; ++i) {
        auto tri_id = children_ids[i];
        for (size_t j = 0; j < 3; ++j) {
            auto v = vertex(*children[i], j);
            vertex_incidence_[v].insert(tri_id);
        }
    }

    // remove current triangle from vertex incidences
    for (size_t j = 0; j < 3; ++j) {
        auto v = vertex(tri, j);
        vertex_incidence_[v].erase(tri_id);
    }

    // vertex_incidence_[mc].insert(children_ids[0]);
    // vertex_incidence_[mc].insert(children_ids[1]);
    // vertex_incidence_[mc].insert(children_ids[3]);
    for (auto tri_id : side_incidence_[side_c]) {
        vertex_incidence_[mc].insert(tri_id);
    }
    // vertex_incidence_[ma].insert(children_ids[1]);
    // vertex_incidence_[ma].insert(children_ids[2]);
    // vertex_incidence_[ma].insert(children_ids[3]);
    for (auto tri_id : side_incidence_[side_a]) {
        vertex_incidence_[ma].insert(tri_id);
    }
    // vertex_incidence_[mb].insert(children_ids[0]);
    // vertex_incidence_[mb].insert(children_ids[2]);
    // vertex_incidence_[mb].insert(children_ids[3]);
    for (auto tri_id : side_incidence_[side_b]) {
        vertex_incidence_[mb].insert(tri_id);
    }

    // add children to vertex incidences
    // vertex_incidence_[a].insert(children_ids[0]);
    // vertex_incidence_[b].insert(children_ids[1]);
    // vertex_incidence_[c].insert(children_ids[2]);

    // update vertex index
    vertex_index_.erase({tri_id, a});
    vertex_index_.erase({tri_id, b});
    vertex_index_.erase({tri_id, c});

    vertex_index_[{children_ids[0], a}] = 0;
    vertex_index_[{children_ids[0], mc}] = 1;
    vertex_index_[{children_ids[0], mb}] = 2;

    vertex_index_[{children_ids[1], mc}] = 0;
    vertex_index_[{children_ids[1], b}] = 1;
    vertex_index_[{children_ids[1], ma}] = 2;

    vertex_index_[{children_ids[2], mb}] = 0;
    vertex_index_[{children_ids[2], ma}] = 1;
    vertex_index_[{children_ids[2], c}] = 2;

    vertex_index_[{children_ids[3], mc}] = 0;
    vertex_index_[{children_ids[3], ma}] = 1;
    vertex_index_[{children_ids[3], mb}] = 2;

    // remove sides of current triangle completely if empty
    if (side_incidence_[side_c].empty()) {
        side_incidence_.erase(side_c);
    }
    if (side_incidence_[side_a].empty()) {
        side_incidence_.erase(side_a);
    }
    if (side_incidence_[side_b].empty()) {
        side_incidence_.erase(side_b);
    }
}
