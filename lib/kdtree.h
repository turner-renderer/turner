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
#include <iostream>
#include <memory>
#include <stack>
#include <vector>

namespace detail {

using TriangleId = uint32_t;
using TriangleIds = std::vector<TriangleId>;

class Node {
public:
    Node(Axis split_axis, float split_pos, Node* left, Node* right)
        : flags_(static_cast<TriangleId>(split_axis) + FLAG_AXIS_X)
        , split_pos_(split_pos)
        , left_or_triangle_ids_(static_cast<void*>(left))
        , right_(right) {}

    explicit Node(const std::vector<TriangleId>& ids) : flags_(ids.size()) {
        TriangleId* triangle_ids = new TriangleId[ids.size()];
        ::memcpy(triangle_ids, ids.data(), ids.size() * sizeof(TriangleId));
        left_or_triangle_ids_ = static_cast<void*>(triangle_ids);
    }

    ~Node() {
        if (is_inner()) {
            if (left()) {
                delete left();
            }
            if (right()) {
                delete right();
            }
        } else {
            delete[] triangle_ids();
        }
    }

    // attributes

    bool is_leaf() const { return flags_ <= MAX_ID; }

    bool is_inner() const { return !is_leaf(); }

    Axis split_axis() const {
        assert(is_inner());

        if (flags_ == FLAG_AXIS_X) {
            return Axis::X;
        } else if (flags_ == FLAG_AXIS_Y) {
            return Axis::Y;
        } else if (flags_ == FLAG_AXIS_Z) {
            return Axis::Z;
        }

        assert(false);
        return Axis::X;
    }

    float split_pos() const {
        assert(is_inner());
        return split_pos_;
    }

    Node* left() const {
        assert(is_inner());
        return reinterpret_cast<Node*>(left_or_triangle_ids_);
    }

    Node* right() const {
        assert(is_inner());
        return right_;
    }

    TriangleId num_tris() const {
        assert(is_leaf());
        return flags_;
    }

    TriangleId* triangle_ids() const {
        assert(is_leaf());
        return reinterpret_cast<TriangleId*>(left_or_triangle_ids_);
    }

    // the last 3 values are reserved for axis description
    static constexpr TriangleId MAX_ID =
        std::numeric_limits<TriangleId>::max() - 3;

    // public interface

    size_t height() const {
        if (is_leaf()) {
            return 0;
        }
        return 1 + std::max(left() ? left()->height() : 0,
                            right() ? right()->height() : 0);
    }

    size_t size() const {
        if (is_leaf()) {
            return num_tris();
        }
        return (left() ? left()->size() : 0) + (right() ? right()->size() : 0);
    }

private:
    static constexpr TriangleId FLAG_AXIS_X = MAX_ID + 1;
    static constexpr TriangleId FLAG_AXIS_Y = MAX_ID + 2;
    static constexpr TriangleId FLAG_AXIS_Z = MAX_ID + 3;

    // If flags_ in [0, MAX_ID], then this node is a leaf node and this
    // member is the number of triangles in the leaf. Otherwise, this node
    // is an inner node. In this case, flags is in [FLAG_AXIS_X,
    // FLAG_AXIS_Y, FLAG_AXIS_Z] and describes the splitting axis.
    TriangleId flags_; // 4 bytes
    // Used only in an inner node
    float split_pos_; // 4 bytes
    // Depending whether this node is an inner node or a leaf, this member
    // is a pointer to the left child or resp. the array containing the
    // triangles.
    void* left_or_triangle_ids_; // 8 bytes
    // Used only in an inner node
    Node* right_; // 8 bytes
                  // == 24 bytes
};

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
 * - a leaf containing two triangles ids. Second, or both of them may be
 *   invalid. A set of triangles belonging to a leaf node is stored as a
 *   contiguous sequence in vector starting with a leaf node and ending with a
 *   node containing a single or no valid triangle ids.
 *
 * The size of the node is 8 bytes. Cf. data_ member for exact memory layout.
 */
class FlatNode {
    constexpr static uint32_t MAX_TRIANGLE_ID = 1 << 30; // excluding
    constexpr static uint32_t INVALID_TRIANGLE_ID = 0xFFFFFFFF >> 2;
    constexpr static uint32_t TYPE_MASK = 3;

public:
    // Inner node containing split axis and pos, and index of the right child
    FlatNode(Axis split_axis, float split_pos, uint32_t right)
        : data_(static_cast<uint64_t>(float_to_uint32(split_pos)) << 32 |
                right << 2 | static_cast<uint32_t>(split_axis)) {
        // we need two bits to store axis and node type
        assert(right < MAX_TRIANGLE_ID);
    }

    // Leaf node containing two ids of triangles (both may be invalid)
    explicit FlatNode(uint32_t triangle_id_a = INVALID_TRIANGLE_ID,
                      uint32_t triangle_id_b = INVALID_TRIANGLE_ID)
        : data_(static_cast<uint64_t>(triangle_id_a) << 32 |
                static_cast<uint64_t>(triangle_id_b << 2 | 3)) {
        assert(triangle_id_a == INVALID_TRIANGLE_ID ||
               triangle_id_a < MAX_TRIANGLE_ID);
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
        return static_cast<uint32_t>(data_ >> 32);
    }

    uint32_t second_triangle_id() const {
        assert(is_leaf());
        return static_cast<uint32_t>((data_ & 0xFFFFFFFF) >> 2);
    }

    bool is_sentinel() const {
        assert(is_leaf());
        return second_triangle_id() == INVALID_TRIANGLE_ID;
    }

    bool is_empty() const {
        assert(is_leaf());
        return data_ == ~(3ull << 62); // all bits except the highest 2 are set
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

} // namespace detail

class KDTreeIntersection;

class KDTree {

    using TriangleIds = detail::TriangleIds;
    using Node = detail::Node;

    friend KDTreeIntersection;

public:
    using TriangleId = detail::TriangleId;

    class OptionalId {
    public:
        OptionalId() : id_(Node::MAX_ID + 1){};
        explicit OptionalId(TriangleId id) : id_(id) {}

        operator bool() const { return id_ <= Node::MAX_ID; }
        operator TriangleId() const {
            assert(*this);
            return id_;
        }
        operator size_t() const {
            assert(*this);
            return id_;
        }
        bool operator==(const OptionalId& other) const {
            return id_ == other.id_;
        }
        bool operator==(const TriangleId& other) const {
            return this->operator bool() && id_ == other;
        }
        bool operator!=(const OptionalId& other) const {
            return !(*this == other);
        }
        bool operator!=(const TriangleId& other) const {
            return !(*this == other);
        }

        friend struct std::hash<OptionalId>;

    private:
        TriangleId id_;
    };

    explicit KDTree(Triangles tris);

    size_t height() const { return root_->height(); }
    size_t size() const { return root_->size(); }
    size_t num_triangles() const { return tris_.size(); }
    const Triangles& triangles() const { return tris_; }
    const Box& box() const { return box_; }
    const Triangle& operator[](const TriangleId id) const { return tris_[id]; }
    const Triangle& at(const TriangleId id) const { return tris_.at(id); }

    static constexpr size_t node_size() { return sizeof(Node); }

private:
    // Helper method for recursive construction
    Node* build(TriangleIds tris, const Box& box);

    // Cf. [WH06], Algorithm 4
    std::tuple<float /*cost*/, Axis /* plane axis */, float /* plane pos */,
               TriangleIds /*left*/, TriangleIds /*right*/>
    find_plane_and_classify(const TriangleIds& tris, const Box& box) const;

    void flatten();

private:
    std::unique_ptr<Node> root_;
    Triangles tris_;
    Box box_;

    // TODO: Describe layout
    std::vector<detail::FlatNode> nodes_; // flatten nodes
};

/**
 * Wraps a KDTree and provides an interface for computing Ray-Triangle
 * intersection.
 */
class KDTreeIntersection {
public:
    using OptionalId = KDTree::OptionalId;
    using TriangleId = KDTree::TriangleId;

    explicit KDTreeIntersection(const KDTree& tree) : tree_(&tree) {}

    const Triangle& operator[](const TriangleId id) const {
        return (*tree_)[id];
    }
    const Triangle& at(const TriangleId id) const { return tree_->at(id); }

    /**
     * Cf. [HH11], Algorithm 2
     *
     * @param  ray     Ray for which the intersection will be computed
     * @param  r       distance from ray to triangle (if intersection
     * exists)
     * @param  a, b    barycentric coordinates of the intersection point
     * @return         optinal id of the triangle hit by the ray
     */
    const OptionalId intersect(const Ray& ray, float& r, float& a, float& b);

    const OptionalId intersect(const Ray& ray) {
        float unused;
        return intersect(ray, unused, unused, unused);
    }

private:
    // Helper method which intersects a triangle ids array with a ray
    const OptionalId intersect(const detail::FlatNode*, const Ray& ray,
                               float& min_r, float& min_s, float& min_t);

private:
    const KDTree* tree_;
    std::stack<
        std::tuple<const detail::FlatNode*, float /*tenter*/, float /*texit*/>>
        stack_;
};

// custom hash for OptionalId
namespace std {

template <> struct hash<KDTree::OptionalId> {
    size_t operator()(const KDTree::OptionalId& id) const {
        return std::hash<detail::TriangleId>()(id.id_);
    }
};

} // namespace std
