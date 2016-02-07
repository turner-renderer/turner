#include "triangle.h"
#include <algorithm>

enum class Axis { X, Y, Z };


// Projection onto axis ax.
float axis_proj(Axis ax, const Vec& v) {
    if (ax == Axis::X) {
        return v.x;
    } else if (ax == Axis::Y) {
        return v.y;
    } else if (ax == Axis::Z) {
        return v.z;
    }
    assert(false);
}

bool axis_comp(Axis ax, const Triangle& tria, const Triangle& trib) {
    auto coord_a = fmin(
        axis_proj(ax, tria.vertices[0]),
        axis_proj(ax, tria.vertices[1]),
        axis_proj(ax, tria.vertices[2]));
    auto coord_b = fmin(
        axis_proj(ax, trib.vertices[0]),
        axis_proj(ax, trib.vertices[1]),
        axis_proj(ax, trib.vertices[2]));
    return coord_a < coord_b;
}


//
// KDTree for D = 3
//
template<unsigned int leaf_capacity=10>
class KDTree {

    struct Node {
        Node(Box bbox, Axis ax, Triangles&& triangles,
                const std::shared_ptr<const Node>& lft,
                const std::shared_ptr<const Node>& rht)
            : bbox(bbox), splitaxis(ax), triangles(triangles)
            , lft_(lft), rht_(rht)
            {}

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
        Box box;
        for (const auto& triangle : triangles) {
            box = box.expand(triangle.bbox());
        }

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

        // do we have to partition at all?
        if (triangles.size() <= leaf_capacity) {
            root_ = std::make_shared<const Node>(
                box, axis, std::move(triangles), nullptr, nullptr);
            return;
        }

        // find median along axis and partition triangles (linear complexity)
        //
        // TODO: Does nth_element invalidate iterators? If not, store mid_it
        // before and use it.
        std::nth_element(
            triangles.begin(),
            triangles.begin() + triangles.size() / 2,
            triangles.end(),
            [axis](const Triangle& tria, const Triangle& trib) {
                return axis_comp(axis, tria, trib);
            });
        auto mid_it = triangles.begin() + triangles.size() / 2;
        auto lft_tree = KDTree(Triangles(triangles.begin(), mid_it));
        auto rht_tree = KDTree(Triangles(mid_it, triangles.end()));

        root_ = std::make_shared<const Node>(
            box, axis, lft_tree.root_, rht_tree.root_);
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

    size_t bytes() const {
        if (empty()) {
            return 0;
        }

        const Node& node = *root_.get();
        return sizeof(node.bbox)
            + sizeof(node.splitaxis)
            + sizeof(Triangle) * node.triangles.size()
            + left().bytes() + right().bytes();
    }

    // algorithms

    bool intersect(
            const Ray& /*ray*/,
            float& /*r*/, float& /*s*/, float& /*t*/) const
    {
        assert(false);  // not implemented
        return true;
    }

private:
    std::shared_ptr<const Node> root_;
};
