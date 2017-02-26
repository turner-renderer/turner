#include "kdtree.h"

#include "clipping.h"
#include "intersection.h"

namespace {

using TriangleId = detail::TriangleId;
using TriangleIds = detail::TriangleIds;

} // namespace anonymous


/**
 * Flatten dynamically allocated KDTree into an array.
 *
 * @param  root node of the KDTree
 * @return array of nodes representing flattened KDTree
 */
mms::vector<mms::Standalone, detail::FlatNodeT<mms::Standalone>> flatten(std::unique_ptr<detail::TreeNode> root) {
    static constexpr uint32_t INVALID_INDEX = 0xFFFFFFFF >> 2;
    const detail::FlatNodeT<mms::Standalone> sentinel(Axis::X, 0, 0);
    mms::vector<mms::Standalone, detail::FlatNodeT<mms::Standalone>> nodes;

    // do DFS through nodes
    std::stack<std::pair<detail::TreeNode*, uint32_t /*parent index*/>> stack;
    stack.emplace(root.get(), INVALID_INDEX);

    while (!stack.empty()) {
        const detail::TreeNode& node = *stack.top().first;
        uint32_t parent_index = stack.top().second;
        stack.pop();

        uint32_t node_index = nodes.size();
        if (parent_index != INVALID_INDEX) {
            // we are in the right node => update parent's reference
            nodes[parent_index].set_right(node_index);
        }

        if (node.is_inner()) {
            // set right to invalid, since we don't know the index yet
            nodes.emplace_back(node.split_axis(), node.split_pos(),
                               INVALID_INDEX);

            // Push right first, since we visit left first.
            stack.emplace(node.right(), node_index);
            stack.emplace(node.left(), INVALID_INDEX);
        } else {
            const TriangleId* triangle_ids = node.triangle_ids();
            // create a leaf node for each pair of triangles
            size_t i = 1;
            for (; i < node.num_tris(); i += 2) {
                nodes.emplace_back(triangle_ids[i - 1], triangle_ids[i]);
            }
            // a triangle left? => add leaf node with a single triangle
            if (i - 1 < node.num_tris()) {
                nodes.emplace_back(triangle_ids[i - 1]);
                // in that case we don't need a sentinel node, since a
                // half-empty node can be used as a sentinel
            } else {
                // add inner node as sentinel
                nodes.push_back(sentinel);
            }
        }
    }

    return nodes;
}

//
// KDTree implementation
//

KDTree::KDTree(Triangles tris, Box box, const mms::vector<mms::Mmapped, detail::FlatNode>* nodes)
    : tris_(std::move(tris)), box_(std::move(box)), nodes_(nodes) {
    assert(tris_.size() > 0);
    assert(tris_.size() < detail::FlatNode::MAX_TRIANGLE_ID);

   // std::vector<TriangleId> ids(tris_.size(), 0);

   // // Compute the bounding box of all triangles and fill in vector of all ids.
   // box_ = tris_.front().bbox();
   // for (size_t i = 1; i < tris_.size(); ++i) {
   //     box_ = box_ + tris_[i].bbox();
   //     ids[i] = i;
   // }

   // KDTreeBuildAlgorithm algo(tris_);
   // mms::vector<mms::Standalone, detail::FlatNodeT<mms::Standalone>> tmp_nodes =
   //     flatten(std::unique_ptr<TreeNode>(algo.build(std::move(ids), box_)));

   // // Save nodes and mmap them.

   // // Serialize
   // std::ofstream out("kdtree_cache");
   // size_t pos = mms::write(out, tmp_nodes);
   // out.close();

   // // mmap data
   // int fd = ::open("kdtree_cache", O_RDONLY);
   // struct stat st;
   // fstat(fd, &st);
   // char* data = (char*) mmap(0, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
   // nodes_ = reinterpret_cast<const mms::vector<mms::Mmapped, detail::FlatNode>*>(data + pos);
}

//
// KDTreeIntersection implementation
//

namespace {

/**
 * Replace all zero coordinates of ray.dir by EPS.
 * @param  ray with direction to fix.
 * @return     new direction.
 */
Vec fix_direction(const Ray& ray) {
    Vec dir = ray.dir;
    for (auto ax : AXES) {
        if (dir[ax] == 0) {
            dir[ax] = EPS;
        }
    }
    return dir;
}

} // namespace anonymous

const KDTreeIntersection::OptionalId
KDTreeIntersection::intersect(const Ray& ray, float& r, float& a, float& b) {
    // A trick to make the traversal robust.
    // Cf. [HH11], p. 5, comment about dir classification and robustness.
    const Ray fixed_ray(ray.pos, fix_direction(ray));

    float tenter, texit;
    if (!intersect_ray_box(fixed_ray, tree_->box(), tenter, texit)) {
        return OptionalId{};
    }

    // Note: No need to clear, since when we leave this function, the stack is
    // always empty.
    assert(stack_.empty());
    const auto* root = &tree_->nodes_->operator[](0);
    stack_.emplace(root, tenter, texit);

    const detail::FlatNode* node;
    OptionalId res;
    r = std::numeric_limits<float>::max();
    while (!stack_.empty()) {
        std::tie(node, tenter, texit) = stack_.top();
        stack_.pop();

        while (node->is_inner()) {
            int ax = static_cast<int>(node->split_axis());
            float split_pos = node->split_pos();

            // t at split
            float t = (split_pos - fixed_ray.pos[ax]) * fixed_ray.invdir[ax];

            // classify near/far with respect to t:
            // left is near if ray.dir[ax] <= 0, else otherwise
            const auto* near = node + 1;
            const auto* far = root + node->right();
            if (fixed_ray.dir[ax] <= 0) {
                std::swap(near, far);
            }

            if (texit < t) {
                node = near;
            } else if (t < tenter) {
                node = far;
            } else {
                stack_.emplace(far, t, texit);
                node = near;
                texit = t;
            }
        }

        assert(node->is_leaf());
        float next_r, next_a, next_b;
        auto next = intersect(node, ray, next_r, next_a, next_b);
        if (next && next_r < r) {
            res = next;
            r = next_r;
            a = next_a;
            b = next_b;
        }
    }

    return res;
}

const KDTreeIntersection::OptionalId
KDTreeIntersection::intersect(const detail::FlatNode* node, const Ray& ray,
                              float& min_r, float& min_s, float& min_t) {
    min_r = std::numeric_limits<float>::max();
    OptionalId res;

    const Triangles& triangles = tree_->tris_;
    auto intersect = [&](uint32_t triangle_id) {
        const auto& tri = triangles[triangle_id];
        float r, s, t;
        bool intersects = intersect_ray_triangle(ray, tri, r, s, t);
        if (intersects && r < min_r) {
            min_r = r;
            min_s = s;
            min_t = t;
            res = OptionalId{triangle_id};
        }
    };

    for (; node->is_leaf(); ++node) {
        intersect(node->first_triangle_id());
        if (!node->has_second_triangle_id()) {
            break;
        }
        intersect(node->second_triangle_id());
    }
    return res;
}
