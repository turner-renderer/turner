#pragma once

#include "triangle.h"
#include "clipping.h"
#include <memory>
#include <algorithm>
#include <unordered_map>
#include <cstdint>
#include <stack>


//
// KDTree for D = 3
//
template<unsigned int LEAF_CAPACITY=10>
class KDTree {

    // TODO: Change the structure of the node s.t. it fits into a cpu cache
    // line of 64 bytes.
    struct Node {
        // a leaf contains triangle
        Node(Box bbox, Axis ax, Triangles&& triangles)
            : bbox(bbox), splitaxis(ax), triangles(triangles)
            {}

        // an inner node contains only bbox and splitting axis
        Node(Box bbox, Axis ax,
                const std::shared_ptr<const Node>& lft,
                const std::shared_ptr<const Node>& rht)
            : bbox(bbox), splitaxis(ax), lft_(lft), rht_(rht)
            {}

        Box bbox;
        Axis splitaxis;
        Triangles triangles;

        std::shared_ptr<const Node> lft_;
        std::shared_ptr<const Node> rht_;
    };

    explicit KDTree(const std::shared_ptr<const Node>& node)
        : root_(node)
        {}

public:
    // Build up the tree.
    KDTree(Triangles triangles) {
        if (triangles.size() == 0) {
            return;
        }

        // compute bounding box of all triangles
        Box box = triangles[0].bbox();
        for (auto it = triangles.begin() + 1; it != triangles.end(); ++it) {
            box = box + it->bbox();
        }
        assert(box.min < box.max);

        // choose the longest axis in the box
        Axis axis;
        float dx = box.max.x - box.min.x;
        float dy = box.max.y - box.min.y;
        float dz = box.max.z - box.min.z;
        if (dx > dy && dx > dz) {
            axis = Axis::X;
        } else if (dy > dz) {
            axis = Axis::Y;
        } else {
            axis = Axis::Z;
        }

        // Do we have to partition at all?
        if (triangles.size() <= LEAF_CAPACITY) {
            root_ = std::make_shared<const Node>(
                box, axis, std::move(triangles));
            return;
        }

        auto split = split_at_spatial_median(axis, box, std::move(triangles));
        root_ = std::make_shared<const Node>(
            box, axis, split.first.root_, split.second.root_);
    }

    // properties

    bool empty() const { return !root_; }
    bool is_leaf() const {return !empty() && !root_->lft_ && !root_->rht_;}

    KDTree left() const { assert(!empty()); return KDTree(root_->lft_); }
    KDTree right() const { assert(!empty()); return KDTree(root_->rht_); }

    Box bbox() const { assert(!empty()); return root_->bbox; }
    Axis splitaxis() const { assert(!empty()); return root_->splitaxis; }
    const Triangles& triangles() const {
        assert(!empty());
        assert(is_leaf());
        return root_->triangles;
    }

    size_t height() const {
        if (empty()) {
            return 0;
        }
        return 1 + std::max(left().height(), right().height());
    }

    // algorithms

    //
    // Args:
    //   r - distance from ray to found triangle
    //   s, t - barycentric coordinates of intersection point
    //
    // Not liking the return pointer used as optional here? Me neither, but I
    // don't have a better idea.
    //
    const Triangle* intersect(
        const Ray& ray, float& r, float& s, float& t) const
    {
        if (empty()) {
            return nullptr;
        }

        if (!ray_box_intersection(ray, bbox())) {
            return nullptr;
        }

        // we are at the bottom in a leaf
        if (is_leaf()) {
            return intersect(ray, triangles(), r, s, t);
        }

        // try to find a triangle in both subtrees
        float left_r = -1, left_s = -1, left_t = -1;
        auto left_tri = left().intersect(ray, left_r, left_s, left_t);

        float right_r = -1, right_s = -1, right_t = -1;
        auto right_tri = right().intersect(ray, right_r, right_s, right_t);

        if (left_tri && (!right_tri || left_r < right_r)) {
            r = left_r;
            s = left_s;
            t = left_t;
            return left_tri;
        } else if (right_tri) {
            r = right_r;
            s = right_s;
            t = right_t;
            return right_tri;
        }

        return nullptr;
    }

    template<unsigned int LEAF_CAPACITY_>
    friend class KDTreeTester;

private:
    std::pair<KDTree, KDTree> split_at_triangles_median(
        const Axis axis, Triangles triangles)
    {
        // Find median along axis and partition triangles (linear complexity).
        // Triangles are sorted along chosen axis at midpoint.
        //
        // TODO: Does nth_element invalidate iterators? If not, store mid_it
        // before and use it.
        std::nth_element(
            triangles.begin(),
            triangles.begin() + triangles.size() / 2,
            triangles.end(),
            [axis](const Triangle& tria, const Triangle& trib) {
                return tria.midpoint()[axis] < trib.midpoint()[axis];
            });
        auto mid_it = triangles.begin() + triangles.size() / 2;

        Triangles rht_triangles(mid_it, triangles.end());
        Triangles& lft_triangles = triangles;
        triangles.erase(mid_it, triangles.end());

        return std::make_pair(
            KDTree(std::move(lft_triangles)),
            KDTree(std::move(rht_triangles)));
    }

    std::pair<KDTree, KDTree> split_at_spatial_median(
        const Axis axis, const Box& bbox, Triangles triangles)
    {
        float min = bbox.min[axis];
        float max = bbox.max[axis];

        Triangles lft_triangles;
        Triangles rht_triangles;

        float axis_midpt;

        // continue splitting to get two real subsets
        while (lft_triangles.empty() || rht_triangles.empty()) {
            if (!lft_triangles.empty()) {
                max = axis_midpt;
                lft_triangles.clear();
            } else if (!rht_triangles.empty()) {
                min = axis_midpt;
                rht_triangles.clear();
            }

            axis_midpt = (min + max) / 2.f;
            for (auto& triangle : triangles) {
                if (triangle.midpoint()[axis] < axis_midpt) {
                    lft_triangles.push_back(triangle);
                } else {
                    rht_triangles.push_back(triangle);
                }
            }
        }

        return std::make_pair(
            std::move(lft_triangles), std::move(rht_triangles));
    }

    const Triangle* intersect(
        const Ray& ray, const Triangles& triangles,
        float& min_r, float& min_s, float& min_t) const
    {
        min_r = -1;
        const Triangle* res = nullptr;
        float r, s, t;
        for (const auto& triangle : triangles) {
            bool intersects = triangle.intersect(ray, r, s, t);
            if (intersects) {
                if (!res || r < min_r) {
                    min_r = r;
                    min_s = s;
                    min_t = t;
                    res = &triangle;
                }
            }
        }

        return res;
    }

    std::shared_ptr<const Node> root_;
};

// ---------------------------------------------------------------------------

//
// Implementation of a fast kd-tree following the article:
//
// "On building fast kd-Trees for Ray Tracing, and on doing that in O(N log N)"
// by Ingo Wald and Vlastimil Havran
//

// Cf. 5.2, Table 1
static constexpr int COST_TRAVERSAL = 15;
static constexpr int COST_INTERSECTION = 20;


struct Plane {
    Axis ax;        // 1 byte
    float coord;    // 4 bytes
                    // = 8 bytes
};

inline std::pair<Box, Box> split(const Box& box, const Plane plane) {
    // assert(box.min[plane.ax] - EPS <= plane.coord);
    // assert(plane.coord <= box.max[plane.ax] + EPS);

    Vec lmax = box.max;
    lmax[plane.ax] = plane.coord;
    Vec rmin = box.min;
    rmin[plane.ax] = plane.coord;

    return {{box.min, lmax}, {rmin, box.max}};
}

// Cost function Bias
inline float lambda(size_t num_ltris, size_t num_rtris) {
    if (num_ltris == 0 || num_rtris == 0) {
        return 0.8f;
    }
    return 1;
}

//
// Cost function of splitting box b at a given plane p
//
// Args:
//   {l,r}area_ratio - ratio of the surface area of the left resp. right
//     box over the box
//   num_{l,r}tris - number of triangles in the left resp. right box
//
// Return:
//   cost to split the box
//
inline float cost(
    float larea_ratio, float rarea_ratio, size_t num_ltris, size_t num_rtris)
{
    return lambda(num_ltris, num_rtris) *
        (COST_TRAVERSAL + COST_INTERSECTION *
            (larea_ratio * num_ltris + rarea_ratio * num_rtris));
}

enum class Dir { LEFT, RIGHT };

//
// SAH function
//
// Args:
//   p - splitting plane
//   box - AABB to split
//   num_{l,r}tris - number of triangles in the left resp. right box produces
//     by splitting `box` at `p`
//   num_ptris - number of triangles lying in the plane `p`
//
// Return:
//   cost to split the box + if the planar triangles should be appended to the
//   lhs or rhs of the box
//
inline std::pair<float /*cost*/, Dir> surface_area_heuristics(
    Plane p, const Box& box,
    size_t num_ltris, size_t num_rtris, size_t num_ptris)
{
    // not splitting at all has infinity cost
    if (box.min[p.ax] == p.coord || box.max[p.ax] == p.coord) {
        return {std::numeric_limits<float>::max(), Dir::LEFT};
    }

    Box lbox, rbox;
    std::tie(lbox, rbox) = split(box, p);
    float area = box.surface_area();
    float larea_ratio = lbox.surface_area()/area;
    float rarea_ratio = rbox.surface_area()/area;

    auto lpcost = cost(
        larea_ratio, rarea_ratio,
        num_ltris + num_ptris, num_rtris);
    auto rpcost = cost(
        larea_ratio, rarea_ratio,
        num_ltris, num_ptris + num_rtris);

    if (lpcost < rpcost) {
        return {lpcost, Dir::LEFT};
    } else {
        return {rpcost, Dir::RIGHT};
    }
}


class FastKDTree {

    using TriangleId = uint32_t;
    using TriangleIds = std::vector<TriangleId>;

private:

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
            // assert(is_inner());

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
            // assert(is_inner());
            return split_pos_;
        }

        Node* left() const {
            // assert(is_inner());
            return reinterpret_cast<Node*>(left_or_triangle_ids_);
        }

        Node* right() const {
            // assert(is_inner());
            return right_;
        }

        TriangleId num_tris() const {
            // assert(is_leaf());
            return flags_;
        }

        TriangleId* triangle_ids() const {
            // assert(is_leaf());
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

        // flags_ == isLeaf, numTris or splitAxis
        TriangleId flags_;            // 4 bytes
        float split_pos_;             // 4 bytes
        void* left_or_triangle_ids_;  // 8 bytes
        Node* right_;                 // 8 bytes
                                      // == 24 bytes
    };

public:

    static constexpr size_t node_size() {
        return sizeof(Node);
    }


    FastKDTree(Triangles tris)
        : tris_(std::move(tris))
    {
        assert(tris_.size() > 0);
        assert(tris_.size() <= Node::MAX_ID);

        std::vector<TriangleId> ids(tris_.size(), 0);

        box_ = tris_.front().bbox();
        for (size_t i = 1; i < tris_.size(); ++i) {
            box_ = box_ + tris_[i].bbox();
            ids[i] = i;
        }

        root_ = std::unique_ptr<Node>(build(std::move(ids), box_));
    }

    // public interface

    size_t height() const {
        return root_->height();
    }

    size_t size() const {
        return root_->size();
    }

    // algorithms

    //
    // Args:
    //   r - distance from ray to triangle (if intersection exists)
    //   s, t - barycentric coordinates of intersection point
    //
    // Not liking the return pointer used as optional here? Me neither.
    //
    // Cf. Algorithm 2 in
    // "Review: Kd-tree Traversal Algorithms for Ray Tracing"
    // by M. Hapala V. Havran
    //
    const Triangle*
    intersect(const Ray& ray, float& r, float& s, float& t) const {
        float tenter, texit;
        if (!ray_box_intersection(ray, box_, tenter, texit)) {
            return nullptr;
        }

        std::stack<std::tuple<Node*, float, float>> stack;
        stack.push(std::make_tuple(root_.get(), tenter, texit));

        Node* node;
        while (!stack.empty()) {
            std::tie(node, tenter, texit) = stack.top();
            stack.pop();

            while (node && node->is_inner()) {
                auto ax = static_cast<int>(node->split_axis());
                auto pos = node->split_pos();

                // t at split
                auto t = (pos - ray.pos[ax]) * ray.invdir[ax];

                // classify near/far with respect to t:
                // left is near if ray.pos < t, else otherwise
                Node* near = node->left();
                Node* far = node->right();
                if (ray.pos[ax] > pos) {
                    std::swap(near, far);
                }

                if (t >= texit || t < 0) {
                    node = near;
                } else if (t <= tenter) {
                    node = far;
                } else {
                    stack.push(std::make_tuple(far, t, texit));
                    node = near;
                    texit = t;
                }
            }

            if (node) {
                assert(node->is_leaf());
                auto res = intersect(
                    node->triangle_ids(), node->num_tris(), ray, r, s, t);
                if (res) {
                    return res;
                }
            }
        }

        return nullptr;
    }


private:

    Node* build(TriangleIds tris, const Box& box) {
        if (tris.size() == 0) {
            return nullptr;
        }

        // to few triangles -> terminate
        if (tris.size() <= 3) {
            return new Node(tris);
        }

        float min_cost;
        Plane plane;
        TriangleIds ltris, rtris;
        std::tie(min_cost, plane, ltris, rtris) = find_plane_and_classify(
            tris, box);

        // clipped everything away
        if (min_cost == 0) {
            return nullptr;
        }

        // automatic termination
        if (min_cost > COST_INTERSECTION * tris.size()) {
            return new Node(tris);
        }

        Box lbox, rbox;
        std::tie(lbox, rbox) = split(box, plane);

        return new Node(plane.ax, plane.coord,
            build(std::move(ltris), lbox),
            build(std::move(rtris), rbox));
    }

    struct Event {
        TriangleId id;
        float point;
        int type;
    };

    static constexpr int STARTING = 2;
    static constexpr int ENDING = 0;
    static constexpr int PLANAR = 1;

    std::tuple<
        float /*cost*/, Plane,
        TriangleIds /*left*/, TriangleIds /*right*/>
    find_plane_and_classify(const TriangleIds& tris, const Box& box) const {

        float min_cost = std::numeric_limits<float>::max();
        Plane min_plane;
        Dir min_side;
        float min_ltris, min_rtris, min_ptris;

        Axis ax = Axis::X;
        std::vector<Event> event_lists[3];

        // generate events
        size_t num_tris = 0;
        for (const auto& id : tris) {

            auto clipped_box = clip_triangle_at_aabb(tris_[id], box);
            if (clipped_box.is_trivial()) {
                continue;
            }
            num_tris += 1;

            for (int k = 0; k < 3; ++k, ++ax) {
                auto& events = event_lists[k];
                if (clipped_box.is_planar(ax)) {
                    events.push_back(
                        Event{id, clipped_box.min[ax], PLANAR});
                } else {
                    events.push_back(
                        Event{id, clipped_box.min[ax], STARTING});
                    events.push_back(
                        Event{id, clipped_box.max[ax], ENDING});
                }
            }
        }

        // sweep
        for (int k = 0; k < 3; ++k, ++ax) {
            auto& events = event_lists[k];
            std::sort(events.begin(), events.end(),
                [](const Event& e1, const Event& e2) {
                    return e1.point < e2.point ||
                        (e1.point == e2.point && e1.type < e2.type);
                });

            int num_ltris = 0, num_ptris = 0, num_rtris = num_tris;

            for (size_t i = 0; i < events.size(); ) {
                auto& event = events[i];

                auto p = event.point;
                int point_starting = 0;
                int point_ending = 0;
                int point_planar = 0;

                while (i < events.size() && events[i].point == p &&
                    events[i].type == ENDING)
                {
                    point_ending += 1;
                    i += 1;
                }
                while (i < events.size() && events[i].point == p &&
                    events[i].type == PLANAR)
                {
                    point_planar += 1;
                    i += 1;
                }
                while (i < events.size() && events[i].point == p &&
                    events[i].type == STARTING)
                {
                    point_starting += 1;
                    i += 1;
                }

                num_ptris = point_planar;
                num_rtris -= point_planar + point_ending;

                Plane plane{ax, p};

                float cost;
                Dir side;
                std::tie(cost, side) = surface_area_heuristics(
                    plane, box, num_ltris, num_rtris, num_ptris);

                if (cost < min_cost) {
                    min_cost = cost;
                    min_plane = plane;
                    min_side = side;
                    min_ltris = num_ltris;
                    min_rtris = num_rtris;
                    min_ptris = num_ptris;
                }

                num_ltris += point_starting + point_planar;
                num_ptris = 0;
            }
        }

        // min_cost is still infty --> terminate
        if (min_cost == std::numeric_limits<float>::max()) {
            return std::make_tuple(
                0, Plane{Axis::X, 0}, TriangleIds(), TriangleIds());
        }

        // -> min_plane, min_side

        // classify

        TriangleIds ltris, rtris;
        ltris.reserve(min_ltris);
        rtris.reserve(min_rtris);

        std::unordered_map<TriangleId, bool> started_left;
        const auto& events = event_lists[static_cast<int>(min_plane.ax)];
        for (const auto& event : events) {
            if (event.point < min_plane.coord) {
                if (event.type == ENDING || event.type == PLANAR) {
                    ltris.push_back(event.id);
                    started_left[event.id] = false;
                } else {
                    started_left[event.id] = true;
                }
            } else if (event.point == min_plane.coord) {
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
            } else {
                if (event.type == STARTING || event.type == PLANAR) {
                    rtris.push_back(event.id);
                } else if (started_left[event.id]) {
                    ltris.push_back(event.id);
                    rtris.push_back(event.id);
                }
            }
        }

        assert(min_ltris + (min_side == Dir::LEFT ? min_ptris : 0)
            == ltris.size());
        assert(min_rtris + (min_side == Dir::RIGHT ? min_ptris : 0)
            == rtris.size());

        return std::make_tuple(
            min_cost, min_plane, std::move(ltris), std::move(rtris));
    }

    // helper method for recursive call with node
    const Triangle* intersect(
        const Node* node, const Box& box,
        const Ray& ray, float& r, float& s, float& t) const
    {
        if (node == nullptr) {
            return nullptr;
        }

        // check if ray intersects the box at all
        if (!ray_box_intersection(ray, box)) {
            return nullptr;
        }

        if (node->is_leaf()) {
            return intersect(
                node->triangle_ids(), node->num_tris(), ray, r, s, t);
        }

        Axis ax = node->split_axis();
        float pos = node->split_pos();

        Vec lmax = box.max;
        lmax[ax] = pos;
        Vec rmin = box.min;
        rmin[ax] = pos;

        Box lbox{box.min, lmax};
        Box rbox{rmin, box.max};
        // Plane plane{node->split_axis(), node->split_pos()};
        // Box lbox, rbox;
        // std::tie(lbox, rbox) = split(box, plane);

        float lr, ls, lt;
        auto ltri = intersect(node->left(), lbox, ray, lr, ls, lt);

        float rr, rs, rt;
        auto rtri = intersect(node->right(), rbox, ray, rr, rs, rt);

        if (ltri && (!rtri || lr < rr)) {
            r = lr;
            s = ls;
            t = lt;
            return ltri;
        } else if (rtri) {
            r = rr;
            s = rs;
            t = rt;
            return rtri;
        }

        return nullptr;
    }

    // helper method which intersects a triangle ids array with a ray
    const Triangle* intersect(
        const TriangleId* ids, uint32_t num_tris,
        const Ray& ray, float& min_r, float& min_s, float& min_t) const
    {
        min_r = std::numeric_limits<float>::max();
        const Triangle* res = nullptr;
        float r, s, t;
        for (uint32_t i = 0; i != num_tris; ++i) {
            const auto& tri = tris_[ids[i]];
            bool intersects = tri.intersect(ray, r, s, t);
            if (intersects) {
                if (r < min_r) {
                    min_r = r;
                    min_s = s;
                    min_t = t;
                    res = &tri;
                }
            }
        }

        return res;
    }

private:
    std::unique_ptr<Node> root_;
    Triangles tris_;
    Box box_;
};
