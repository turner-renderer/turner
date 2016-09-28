//
// Kd-tree structure for storing a set of triangles and providing a fast
// intersection lookup.
//
// Implementation of the construction of fast kd-tree follows the article:
//
// "On building fast kd-Trees for Ray Tracing, and on doing that in O(N log N)"
// by Ingo Wald and Vlastimil Havran
// [WH06]
//
// Cf. Algorithm 4, i.e. we build up the kd-tree in O(N log^2 N), however the
// resulting kd-tree is the same as if it were built with Algorithm 5. TODO:
// Replace by Algorithm 5.
//
// Implementation of the intersection lookup follows the article:
// "Review: Kd-tree Traversal Algorithms for Ray Tracing"
// by M. Hapala V. Havran
// [HH11]
//

#pragma once

#include "triangle.h"
#include <cstdint>
#include <vector>
#include <memory>


namespace impl {

using TriangleId = uint32_t;
using TriangleIds = std::vector<TriangleId>;

/**
 * Node used for building up kd-tree.
 */
class Node {
public:
    Node(Axis split_axis, float split_pos, Node* left, Node* right)
    : flags_(static_cast<TriangleId>(split_axis) + FLAG_AXIS_X)
    , split_pos_(split_pos)
    , left_or_triangle_ids_(static_cast<void*>(left))
    , right_(right)
    {}

    explicit Node(const std::vector<TriangleId>& ids)
    : flags_(ids.size())
    {
        TriangleId* triangle_ids = new TriangleId[ids.size()];
        ::memcpy(
            triangle_ids, ids.data(), ids.size() * sizeof(TriangleId));
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

    bool is_leaf() const {
        return flags_ <= MAX_ID;
    }

    bool is_inner() const {
        return !is_leaf();
    }

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
    static constexpr TriangleId MAX_ID = \
        std::numeric_limits<TriangleId>::max() - 3;

    // public interface

    size_t height() const {
        if (is_leaf()) {
            return 0;
        }
        return 1 + std::max(
            left() ? left()->height() : 0,
            right() ? right()->height() : 0);
    }

    size_t size() const {
        if (is_leaf()) {
            return num_tris();
        }
        return (left() ? left()->size() : 0) +
            (right() ? right()->size() : 0);
    }

private:
    static constexpr TriangleId FLAG_AXIS_X = MAX_ID + 1;
    static constexpr TriangleId FLAG_AXIS_Y = MAX_ID + 2;
    static constexpr TriangleId FLAG_AXIS_Z = MAX_ID + 3;

    // If flags_ in [0, MAX_ID], then this node is a leaf node and this
    // member is the number of triangles in the leaf. Otherwise, this node
    // is an inner node. In this case, flags is in [FLAG_AXIS_X,
    // FLAG_AXIS_Y, FLAG_AXIS_Z] and describes the splitting axis.
    TriangleId flags_;            // 4 bytes
    // Used only in an inner node
    float split_pos_;             // 4 bytes
    // Depending whether this node is an inner node or a leaf, this member
    // is a pointer to the left child or resp. the array containing the
    // triangles.
    void* left_or_triangle_ids_;  // 8 bytes
    // Used only in an inner node
    Node* right_;                 // 8 bytes
                                  // == 24 bytes
};

/**
 * Node used as internal storage structure in flattened kd-tree.
 */
class FlatNode {
public:
    // Create inner node
    FlatNode(Axis split_axis, float split_pos, TriangleId left_index,
            TriangleId right_index)
        : flags_(static_cast<TriangleId>(split_axis) + FLAG_AXIS_X)
        , split_pos_(split_pos)
        , left_or_triangle_ids_(left_index)
        , right_(right_index)
    {}

    // Create leaf node
    FlatNode(size_t num_triangles, TriangleId index)
        : flags_(num_triangles)
        , split_pos_(0.f) // unused
        , left_or_triangle_ids_(index)
        , right_(0) // unused
    {}

    // attributes

    bool is_leaf() const {
        return flags_ <= MAX_ID;
    }

    bool is_inner() const {
        return !is_leaf();
    }

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
    }

    float split_pos() const {
        assert(is_inner());
        return split_pos_;
    }

    TriangleId left() const {
        assert(is_inner());
        return left_or_triangle_ids_;
    }

    void set_left(TriangleId id) {
        assert(is_inner());
        left_or_triangle_ids_ = id;
    }

    TriangleId right() const {
        assert(is_inner());
        return right_;
    }

    void set_right(TriangleId id) {
        assert(is_inner());
        right_ = id;
    }

    size_t num_tris() const {
        assert(is_leaf());
        return flags_;
    }

    TriangleId triangle_ids() const {
        assert(is_leaf());
        return left_or_triangle_ids_;
    }

    void set_triangle_ids(TriangleId index) {
        assert(is_leaf());
        left_or_triangle_ids_ = index;
    }

    // the last 3 values are reserved for axis description
    static constexpr TriangleId MAX_ID = \
        std::numeric_limits<TriangleId>::max() - 3;

private:
    static constexpr TriangleId FLAG_AXIS_X = MAX_ID + 1;
    static constexpr TriangleId FLAG_AXIS_Y = MAX_ID + 2;
    static constexpr TriangleId FLAG_AXIS_Z = MAX_ID + 3;

    // If flags_ in [0, MAX_ID], then this node is a leaf node and this
    // member is the number of triangles in the leaf. Otherwise, this node
    // is an inner node. In this case, flags is in [FLAG_AXIS_X,
    // FLAG_AXIS_Y, FLAG_AXIS_Z] and describes the splitting axis.
    TriangleId flags_;            // 4 bytes
    // Used only in an inner node
    float split_pos_;             // 4 bytes
    // Depending whether this node is an inner node or a leaf, this member
    // is an index to the left child (0 if no child) or resp. to the index in
    // tris_ids_ vector.          // 4 bytes
    TriangleId left_or_triangle_ids_;
    // Used only in an inner node; index to the right child; 0 if no child.
    TriangleId right_;            // 4 bytes
                                  // == 16 bytes
};

} // namespace impl

class KDTree {

    using TriangleIds = impl::TriangleIds;
    using Node = impl::Node;
    using FlatNode = impl::FlatNode;

public:
    using TriangleId = impl::TriangleId;

    class OptionalId {
    public:
        OptionalId() : id_(FlatNode::MAX_ID + 1) {};
        explicit OptionalId(TriangleId id) : id_(id) {}

        operator bool() const { return id_ <= FlatNode::MAX_ID; }
        operator TriangleId() const { assert(*this); return id_; }
        operator size_t() const { assert(*this); return id_; }
        bool operator==(const OptionalId& other) const {
            return id_ == other.id_;
        }
        bool operator==(const TriangleId& id) const {
            return static_cast<bool>(id_) && id_ == id;
        }

        friend struct std::hash<OptionalId>;

    private:
        TriangleId id_;
    };

    explicit KDTree(Triangles tris);

    size_t height() const { return nodes_.size() > 1 ? node_height(1) : 0; }
    size_t size() const { return tris_ids_.size(); }
    size_t num_triangles() const { return tris_.size(); }
    const Triangle& operator[](const TriangleId id) const {
        return tris_[id];
    }
    const Triangle& at(const TriangleId id) const {
        return tris_.at(id);
    }

    static constexpr size_t node_size() { return sizeof(FlatNode); }

    //
    // Cf. [HH11], Algorithm 2
    //
    // Args:
    //   r - distance from ray to triangle (if intersection exists)
    //   s, t - barycentric coordinates of intersection point
    //
    const OptionalId
    intersect(const Ray& ray, float& r, float& s, float& t) const;

private:
    // Helper method for recursive construction
    Node* build(TriangleIds tris, const Box& box);

    // Cf. [WH06], Algorithm 4
    std::tuple<
        float /*cost*/, Axis /* plane axis */, float /* plane pos */,
        TriangleIds /*left*/, TriangleIds /*right*/>
    find_plane_and_classify(const TriangleIds& tris, const Box& box) const;

    // Helper method which intersects a triangle ids array with a ray
    const OptionalId intersect(
        TriangleId triangle_id_index, uint32_t num_tris,
        const Ray& ray, float& min_r, float& min_s, float& min_t) const;

    // Helper method to compute the height of a node given by index.
    size_t node_height(const TriangleId index) const;

private:
    // index 0 is sentinel for invalid data
    // index 1 is root
    std::vector<impl::FlatNode> nodes_;
    std::vector<TriangleId> tris_ids_;
    Triangles tris_;
    Box box_;
};


// custom hash for OptionalId
namespace std {

template<>
struct std::hash<KDTree::OptionalId> {
    size_t operator()(const KDTree::OptionalId& id) const;
};

} // namespace std
