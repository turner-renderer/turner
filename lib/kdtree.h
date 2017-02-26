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

#include "clipping.h"
#include "triangle.h"

#include <mms/vector.h>
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
template<class P>
class FlatNodeT {
    constexpr static uint32_t TYPE_MASK = 3;

public:
    constexpr static uint32_t MAX_TRIANGLE_ID = 1 << 30; // excluding
    constexpr static uint32_t INVALID_TRIANGLE_ID = 0xFFFFFFFF >> 2;

public:
    // Inner node containing split axis and pos, and index of the right child
    FlatNodeT(Axis split_axis, float split_pos, uint32_t right)
        : data_(static_cast<uint64_t>(float_to_uint32(split_pos)) << 32 |
                right << 2 | static_cast<uint32_t>(split_axis)) {
        // we need two bits to store axis and node type
        assert(right < MAX_TRIANGLE_ID);
    }

    // Leaf node containing two ids of triangles (both may be invalid)
    explicit FlatNodeT(uint32_t triangle_id_a,
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

    // Expose struct's fields to mms
    template<class A> void traverseFields(A a) const { a(data_); }

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

// Defines an alias for memory mapped nodes that should always be used.
using FlatNode = FlatNodeT<mms::Mmapped>;

/**
 * Dynamically allocated KDTree node.
 */
class TreeNode {
public:
    TreeNode(Axis split_axis, float split_pos, TreeNode* left, TreeNode* right)
        : flags_(static_cast<TriangleId>(split_axis) + FLAG_AXIS_X)
        , split_pos_(split_pos)
        , left_or_triangle_ids_(static_cast<void*>(left))
        , right_(right) {}

    explicit TreeNode(const std::vector<TriangleId>& ids) : flags_(ids.size()) {
        TriangleId* triangle_ids = new TriangleId[ids.size()];
        ::memcpy(triangle_ids, ids.data(), ids.size() * sizeof(TriangleId));
        left_or_triangle_ids_ = static_cast<void*>(triangle_ids);
    }

    ~TreeNode() {
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

    TreeNode* left() const {
        assert(is_inner());
        return reinterpret_cast<TreeNode*>(left_or_triangle_ids_);
    }

    TreeNode* right() const {
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
    TreeNode* right_; // 8 bytes
                      // == 24 bytes
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

/**
 * Flatten dynamically allocated KDTree into an array.
 *
 * @param  root node of the KDTree
 * @return array of nodes representing flattened KDTree
 */
mms::vector<mms::Standalone, detail::FlatNodeT<mms::Standalone>> flatten(std::unique_ptr<detail::TreeNode> root);

class KDTreeIntersection;

class KDTree {
    friend KDTreeIntersection;

public:
    using TriangleId = detail::TriangleId;

    explicit KDTree(Triangles tris, Box box);
    explicit KDTree(Triangles tris, Box box,
        const mms::vector<mms::Mmapped, detail::FlatNode>* nodes);

    size_t height() const {
        using Node = detail::FlatNode;
        std::stack<std::pair<const Node*, uint32_t /* level */>> stack;

        const Node* root = &nodes_->operator[](0);
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
    size_t num_nodes() const { return nodes_->size(); }
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
     const mms::vector<mms::Mmapped, detail::FlatNode>* nodes_;
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

using TriangleId = detail::TriangleId;
using TriangleIds = detail::TriangleIds;

/**
 * Self contained implementation of Algorithm 4 from:
 *
 * "On building fast kd-Trees for Ray Tracing, and on doing that in O(N log N)"
 * by Ingo Wald and Vlastimil Havran
 * [WH06]
 *
 * Builds up a kd-tree in O(N log^2 N), however the resulting kd-tree is the
 * same as if it were built with Algorithm 5.
 *
 * TODO: Replace by Algorithm 5.
 */
class KDTreeBuildAlgorithm {
public:
    KDTreeBuildAlgorithm(const Triangles& triangles) : triangles_(&triangles) {}

    detail::TreeNode* build(TriangleIds tris, const Box& box) {
        using Node = detail::TreeNode;

        if (tris.size() == 0) {
            return nullptr;
        }

        // to few triangles -> terminate
        if (tris.size() <= 3) {
            return new Node(tris);
        }

        // box too small -> no need to split further -> terminate
        if (box.surface_area() == 0) {
            return new Node(tris);
        }

        float min_cost;
        Axis plane_ax;
        float plane_pos; // plane
        TriangleIds ltris, rtris;
        std::tie(min_cost, plane_ax, plane_pos, ltris, rtris) =
            find_plane_and_classify(tris, box);

        // automatic termination
        // remove lambda factor from cost again, otherwise we may stuck in an
        // empty space split forever
        if (COST_INTERSECTION * tris.size() *
                lambda(ltris.size(), rtris.size()) <
            min_cost) {
            return new Node(tris);
        }

        Box lbox, rbox;
        std::tie(lbox, rbox) = box.split(plane_ax, plane_pos);

        Node* left = build(std::move(ltris), lbox);
        Node* right = build(std::move(rtris), rbox);
        assert(left || right);

        if (!left) {
            return right;
        } else if (!right) {
            return left;
        }
        return new Node(plane_ax, plane_pos, left, right);
    }

private:
    // Cf. [WH06], 5.2, Table 1
    static constexpr int COST_TRAVERSAL = 15;
    static constexpr int COST_INTERSECTION = 20;

    // Cost function bias
    float lambda(size_t num_ltris, size_t num_rtris) const {
        if (num_ltris == 0 || num_rtris == 0) {
            return 0.8f;
        }
        return 1;
    }

    /**
     * Cost function of splitting box b at a given plane p
     *
     * @param  {l,r}area_ratio ratio of the surface area of the left resp.
     *         right box over the box
     * @param  num_{l,r}tris   number of triangles in the left resp. right box
     * @return cost to split the box
     */
    float cost(float larea_ratio, float rarea_ratio, size_t num_ltris,
               size_t num_rtris) const {
        return lambda(num_ltris, num_rtris) *
               (COST_TRAVERSAL +
                COST_INTERSECTION *
                    (larea_ratio * num_ltris + rarea_ratio * num_rtris));
    }

    enum class Dir { LEFT, RIGHT };

    /**
     * SAH function
     *
     * @param  ax, pos       splitting plane
     * @param  box           AABB to split
     * @param  num_{l,r}tris number of triangles in the left resp. right box
     *                       produces by splitting `box` at `p`
     * @param  num_ptris     number of triangles lying in the plane `p`
     * @return cost to split the box + whether the planar triangles should be
     *         appended to the lhs or rhs of the box.
     */
    std::pair<float /*cost*/, Dir>
    surface_area_heuristics(Axis ax, float pos, const Box& box,
                            size_t num_ltris, size_t num_rtris,
                            size_t num_ptris) const {
        Box lbox, rbox;
        std::tie(lbox, rbox) = box.split(ax, pos);
        float area = box.surface_area();
        float larea_ratio = lbox.surface_area() / area;
        float rarea_ratio = rbox.surface_area() / area;

        auto left_planar_cost =
            cost(larea_ratio, rarea_ratio, num_ltris + num_ptris, num_rtris);
        auto right_planar_cost =
            cost(larea_ratio, rarea_ratio, num_ltris, num_ptris + num_rtris);

        if (left_planar_cost < right_planar_cost) {
            return {left_planar_cost, Dir::LEFT};
        } else {
            return {right_planar_cost, Dir::RIGHT};
        }
    }

    /**
     * [WH06], Algorithm 4
     */
    std::tuple<float /*cost*/, Axis /* plane axis */, float /* plane pos */,
               TriangleIds /*left*/, TriangleIds /*right*/>
    find_plane_and_classify(const TriangleIds& tris, const Box& box) const {
        // The box should have some surface, otherwise the surface area
        // heuristics
        // does not make any sense.
        assert(box.surface_area() != 0);

        // auxiliary event structure
        static constexpr int STARTING = 2;
        static constexpr int ENDING = 0;
        static constexpr int PLANAR = 1;

        struct Event {
            TriangleId id;
            float point;
            // auxiliary point
            // for type == ENDING, point_aux is the starting point
            // for type == STARTNG, point_aux is the ending point
            // for type == PLANAR, points_aux is the same point as `point`
            float point_aux;
            int type;
        };

        // PERF: move out in front of recursion
        std::vector<Event> event_lists[AXES.size()];

        // generate events
        size_t num_tris = 0;
        for (const auto& id : tris) {
            auto clipped_box = clip_triangle_at_aabb((*triangles_)[id], box);
            if (clipped_box.is_trivial()) {
                continue;
            }
            num_tris += 1;

            for (auto ax : AXES) {
                auto& events = event_lists[static_cast<int>(ax)];
                if (clipped_box.is_planar(ax)) {
                    events.emplace_back(Event{id, clipped_box.min[ax],
                                              clipped_box.min[ax], PLANAR});
                } else {
                    events.emplace_back(Event{id, clipped_box.min[ax],
                                              clipped_box.max[ax], STARTING});
                    events.emplace_back(Event{id, clipped_box.max[ax],
                                              clipped_box.min[ax], ENDING});
                }
            }
        }

        // all clipped?
        if (num_tris == 0) {
            return std::make_tuple(std::numeric_limits<float>::max(), Axis::X,
                                   0, TriangleIds(), TriangleIds());
        }

        // sweep for min_cost
        float min_cost = std::numeric_limits<float>::max();
        Axis min_plane_ax = Axis::X;
        float min_plane_pos = 0;
        Dir min_side = Dir::LEFT;
        // we also store those for asserts below
        size_t min_ltris = 0;
        size_t min_rtris = 0;
        size_t min_ptris = 0;
        UNUSED(min_ptris);

        for (auto ax : AXES) {
            auto& events = event_lists[static_cast<int>(ax)];
            std::sort(events.begin(), events.end(),
                      [](const Event& e1, const Event& e2) {
                          return e1.point < e2.point ||
                                 (e1.point == e2.point && e1.type < e2.type);
                      });

            size_t num_ltris = 0, num_ptris = 0, num_rtris = num_tris;

            for (size_t i = 0; i < events.size();) {
                auto& event = events[i];

                auto p = event.point;
                int point_starting = 0;
                int point_ending = 0;
                int point_planar = 0;

                while (i < events.size() && events[i].point == p &&
                       events[i].type == ENDING) {
                    point_ending += 1;
                    i += 1;
                }
                while (i < events.size() && events[i].point == p &&
                       events[i].type == PLANAR) {
                    point_planar += 1;
                    i += 1;
                }
                while (i < events.size() && events[i].point == p &&
                       events[i].type == STARTING) {
                    point_starting += 1;
                    i += 1;
                }

                num_ptris = point_planar;
                num_rtris -= point_planar + point_ending;

                float cost;
                Dir side;
                std::tie(cost, side) = surface_area_heuristics(
                    ax, p, box, num_ltris, num_rtris, num_ptris);

                if (cost < min_cost) {
                    min_cost = cost;
                    min_plane_ax = ax;
                    min_plane_pos = p;
                    min_side = side;
                    min_ltris = num_ltris;
                    min_rtris = num_rtris;
                    min_ptris = num_ptris;
                }

                num_ltris += point_starting + point_planar;
                num_ptris = 0;
            }
        } // -> min_plane, min_side

        assert(min_cost < std::numeric_limits<float>::max());

        // classify
        TriangleIds ltris, rtris;
        ltris.reserve(min_ltris);
        rtris.reserve(min_rtris);

        const auto& events = event_lists[static_cast<int>(min_plane_ax)];
        for (const auto& event : events) {
            if (event.point < min_plane_pos) {
                if (event.type == ENDING || event.type == PLANAR) {
                    ltris.push_back(event.id);
                } else if (min_plane_pos < event.point_aux) {
                    // STARTING before and ENDING after min_plane
                    ltris.push_back(event.id);
                    rtris.push_back(event.id);
                }
            } else if (event.point == min_plane_pos) {
                if (event.type == ENDING) {
                    ltris.push_back(event.id);
                } else if (event.type == PLANAR) {
                    if (min_side == Dir::LEFT) {
                        ltris.push_back(event.id);
                    } else {
                        rtris.push_back(event.id);
                    }
                } else {
                    rtris.push_back(event.id);
                }
            } else if (event.type == STARTING || event.type == PLANAR) {
                rtris.push_back(event.id);
            }
        }

        assert(min_ltris + (min_side == Dir::LEFT ? min_ptris : 0) ==
               ltris.size());
        assert(min_rtris + (min_side == Dir::RIGHT ? min_ptris : 0) ==
               rtris.size());

        return std::make_tuple(min_cost, min_plane_ax, min_plane_pos,
                               std::move(ltris), std::move(rtris));
    }

private:
    const Triangles* triangles_;
};

// custom hash for OptionalId
namespace std {

template <> struct hash<KDTreeIntersection::OptionalId> {
    size_t operator()(const KDTreeIntersection::OptionalId& id) const {
        return std::hash<detail::TriangleId>()(id.id_);
    }
};

} // namespace std
