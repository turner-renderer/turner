/**
 * Kd-tree structure for storing a set of triangles and providing a fast
 * intersection lookup.
 *
 * Implementation of the construction of fast kd-tree follows the article:
 *
 * "On building fast kd-Trees for Ray Tracing, and on doing that in O(N log N)"
 * by Ingo Wald and Vlastimil Havran
 * [WH06]
 *
 * Cf. Algorithm 4, i.e. we build up the kd-tree in O(N log^2 N), however the
 * resulting kd-tree is the same as if it were built with Algorithm 5. TODO:
 * Replace by Algorithm 5.
 *
 * Implementation of the intersection lookup follows the article:
 * "Review: Kd-tree Traversal Algorithms for Ray Tracing"
 * by M. Hapala V. Havran
 * [HH11]
 */

#pragma once

#include "triangle.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <stack>
#include <vector>

namespace detail {

using TriangleId = uint32_t;
using TriangleIds = std::vector<TriangleId>;

inline uint32_t float_to_uint32(float val) {
    uint32_t result;
    std::memcpy(&result, &val, sizeof(val));
    return result;
}

inline float uint32_to_float(uint32_t val) {
    float result;
    std::memcpy(&result, &val, sizeof(val));
    return result;
}

/**
 * Node in a flattened KDTree.
 *
 * This is either:
 * - an inner node containing a split axis and position, and the index of the
 *   right child. The left child is not stored explicitly; it is stored as the
 *   next neighbor in the vector containing nodes. Or,
 * - a leaf containing two triangles ids. Second, may be invalid. A set of
 *   triangles belonging to a leaf node is stored as a contiguous sequence in
 *   vector starting with a leaf node and ending with an inner node.
 *
 * The size of the node is 8 bytes. Cf. data_ member for exact memory layout.
 */
class FlatNode {
    constexpr static uint32_t TYPE_MASK = 3;

public:
    constexpr static uint32_t MAX_TRIANGLE_ID = 1 << 30; // excluding
    constexpr static uint32_t INVALID_TRIANGLE_ID = 0xFFFFFFFF >> 2;

public:
    // Inner node containing split axis and pos, and index of the right child
    FlatNode(Axis split_axis, float split_pos, uint32_t right)
        : data_(static_cast<uint64_t>(float_to_uint32(split_pos)) << 32 |
                right << 2 | static_cast<uint32_t>(split_axis)) {
        // we need two bits to store axis and node type
        assert(right < MAX_TRIANGLE_ID);
    }

    // Leaf node containing two ids of triangles (both may be invalid)
    explicit FlatNode(uint32_t triangle_id_a,
                      uint32_t triangle_id_b = INVALID_TRIANGLE_ID)
        : data_(static_cast<uint64_t>(triangle_id_a) << 32 |
                static_cast<uint64_t>(triangle_id_b << 2 | 3)) {
        assert(triangle_id_a < MAX_TRIANGLE_ID);
        assert(triangle_id_b == INVALID_TRIANGLE_ID ||
               triangle_id_b < MAX_TRIANGLE_ID);
    }

    // attributes

    bool is_leaf() const { return (data_ & TYPE_MASK) == 3; }
    bool is_inner() const { return !is_leaf(); }

    // inner node attributes

    Axis split_axis() const {
        assert(is_inner());
        return static_cast<Axis>(data_ & TYPE_MASK);
    }

    float split_pos() const {
        assert(is_inner());
        return uint32_to_float(data_ >> 32);
    }

    uint32_t right() const {
        assert(is_inner());
        return static_cast<uint32_t>(data_) >> 2;
    }

    // Since right nodes are create after parent nodes, we need a setter to
    // update the index of the right node after creation.
    void set_right(uint32_t value) {
        data_ = (data_ & 0xFFFFFFFF00000000) | value << 2 |
                static_cast<uint32_t>(split_axis());
    }

    // leaf node attributes

    uint32_t first_triangle_id() const {
        assert(is_leaf());
        return data_ >> 32;
    }

    bool has_second_triangle_id() const {
        assert(is_leaf());
        return ~static_cast<uint32_t>(data_ & 0xFFFFFFFF);
    }

    uint32_t second_triangle_id() const {
        assert(is_leaf());
        return (data_ & 0xFFFFFFFF) >> 2;
    }

private:
    /**
     * Memory layout:
     * [  32 bits] [30 bits] [    2 bits]  = 8 bytes
     * [split_pos] [  right] [split_axis]  inner node
     * [   tri_id] [ tri_id] [       1 1]  leaf
     *
     * 2 last bits describe the node type and splitting axis:
     * 0 1  inner with X-axis
     * 1 1  inner with Y-axis
     * 1 0  inner with Z-axis
     * 1 1  leaf
     */
    uint64_t data_;
};

class OptionalId {
public:
    OptionalId() : id_(FlatNode::MAX_TRIANGLE_ID){};
    explicit OptionalId(TriangleId id) : id_(id) {}

    operator bool() const { return id_ < FlatNode::MAX_TRIANGLE_ID; }
    operator TriangleId() const {
        assert(*this);
        return id_;
    }
    operator size_t() const {
        assert(*this);
        return id_;
    }
    bool operator==(const OptionalId& other) const { return id_ == other.id_; }
    bool operator==(const TriangleId& other) const {
        return this->operator bool() && id_ == other;
    }
    bool operator!=(const OptionalId& other) const { return !(*this == other); }
    bool operator!=(const TriangleId& other) const { return !(*this == other); }

    friend struct std::hash<OptionalId>;

private:
    TriangleId id_;
};

} // namespace detail

class KDTreeIntersection;

class KDTree {
    friend KDTreeIntersection;

public:
    using TriangleId = detail::TriangleId;

    explicit KDTree(Triangles tris);

    size_t height() const {
        using Node = detail::FlatNode;
        std::stack<std::pair<const Node*, uint32_t /* level */>> stack;
        const Node* root = nodes_.data();
        stack.emplace(root, 0);
        size_t height = 0;
        while (!stack.empty()) {
            size_t level = stack.top().second;
            if (level > height) {
                height = level;
            }

            const Node* node = stack.top().first;
            stack.pop();

            if (node->is_inner()) {
                stack.emplace(node + 1, level + 1);
                stack.emplace(root + node->right(), level + 1);
            }
        }
        return height;
    }
    size_t num_nodes() const { return nodes_.size(); }
    size_t num_triangles() const { return tris_.size(); }
    const Triangles& triangles() const { return tris_; }
    const Box& box() const { return box_; }
    const Triangle& operator[](const TriangleId id) const { return tris_[id]; }
    const Triangle& at(const TriangleId id) const { return tris_.at(id); }

    static constexpr size_t node_size() { return sizeof(detail::FlatNode); }

private:
    Triangles tris_;
    Box box_;

    /**
     * Layout:
     *
     * We have 2 types of nodes (cf. Node): inner nodes and leaf nodes. All
     * nodes are stored in the DFS order. An inner node has its left child as
     * the next node, and stores an index to its right child. A leaf node
     * contains at most two valid references into triangles_ array. All triangle
     * ids are read from the first leaf until the first sentinel. A sentinel is
     * either an inner node without a parent, or a leaf node containing a single
     * triangle.
     *
     * Cf. possible layout of the array:
     *
     * [] - a node, i - inner node, l - leaf, / - sentinel
     * [i] [i] [l l] [/] [  l] [l /] [  l] [/]
     * [0] [1] [2 3] [/] [4 5] [6  ] [7 8] [/]
     *
     * representing the following tree:
     *
     *         0
     *        / \
     *       /   \
     *      1    [7 8]
     *     / \
     *    /   \
     * [2 3]  [4 5 6]
     */
    std::vector<detail::FlatNode> nodes_;
};

/**
 * Wraps a KDTree and provides an interface for computing Ray-Triangle
 * intersection.
 */
class KDTreeIntersection {
public:
    using TriangleId = KDTree::TriangleId;
    using OptionalId = detail::OptionalId;

    explicit KDTreeIntersection(const KDTree& tree) : tree_(&tree) {}

    const Triangle& operator[](const TriangleId id) const {
        return (*tree_)[id];
    }
    const Triangle& at(const TriangleId id) const { return tree_->at(id); }

    /**
     * Cf. [HH11], Algorithm 2
     *
     * @param  ray   Ray for which the intersection will be computed
     * @param  r     distance from ray to triangle (if intersection
     *               exists)
     * @param  a, b  barycentric coordinates of the intersection point
     * @return       optional id of the triangle hit by the ray
     */
    const OptionalId intersect(const Ray& ray, float& r, float& a, float& b);

    const OptionalId intersect(const Ray& ray) {
        float unused;
        return intersect(ray, unused, unused, unused);
    }

private:
    // Helper method which intersects triangles from consecutive nodes (starting
    // at node) until we reach an inner node.
    const OptionalId intersect(const detail::FlatNode* node, const Ray& ray,
                               float& min_r, float& min_s, float& min_t);

private:
    const KDTree* tree_;
    std::stack<
        std::tuple<const detail::FlatNode*, float /*tenter*/, float /*texit*/>>
        stack_;
};

// custom hash for OptionalId
namespace std {

template <> struct hash<KDTreeIntersection::OptionalId> {
    size_t operator()(const KDTreeIntersection::OptionalId& id) const {
        return std::hash<detail::TriangleId>()(id.id_);
    }
};

} // namespace std
