#include "kdtree.h"

#include "clipping.h"
#include "intersection.h"

#include <stack>

namespace {

// Cf. [WH06], 5.2, Table 1
static constexpr int COST_TRAVERSAL = 15;
static constexpr int COST_INTERSECTION = 20;

// Cost function bias
float lambda(size_t num_ltris, size_t num_rtris) {
    if (num_ltris == 0 || num_rtris == 0) {
        return 0.8f;
    }
    return 1;
}

/**
 * Cost function of splitting box b at a given plane p
 *
 * Args:
 *   {l,r}area_ratio - ratio of the surface area of the left resp. right
 *     box over the box
 *   num_{l,r}tris - number of triangles in the left resp. right box
 *
 * Return:
 *   cost to split the box
 */
float cost(float larea_ratio, float rarea_ratio, size_t num_ltris,
           size_t num_rtris) {
    return lambda(num_ltris, num_rtris) *
           (COST_TRAVERSAL +
            COST_INTERSECTION *
                (larea_ratio * num_ltris + rarea_ratio * num_rtris));
}

enum class Dir { LEFT, RIGHT };

/**
 * SAH function
 *
 * Args:
 *   ax, pos: splitting plane
 *   box: AABB to split
 *   num_{l,r}tris: number of triangles in the left resp. right box produces
 *     by splitting `box` at `p`
 *   num_ptris: number of triangles lying in the plane `p`
 *
 * Return:
 *   cost to split the box + wether the planar triangles should be appended to
 *   the lhs or rhs of the box
 */
std::pair<float /*cost*/, Dir>
surface_area_heuristics(Axis ax, float pos, const Box& box, size_t num_ltris,
                        size_t num_rtris, size_t num_ptris) {
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
} // namespace anonymous

KDTree::KDTree(Triangles tris) : tris_(std::move(tris)) {
    assert(tris_.size() > 0);
    assert(tris_.size() <= Node::MAX_ID);

    std::vector<TriangleId> ids(tris_.size(), 0);

    box_ = tris_.front().bbox();
    for (size_t i = 1; i < tris_.size(); ++i) {
        box_ = box_ + tris_[i].bbox();
        ids[i] = i;
    }

    root_ = std::unique_ptr<Node>(build(std::move(ids), box_));
    flatten();
}

KDTree::Node* KDTree::build(KDTree::TriangleIds tris, const Box& box) {
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
    if (COST_INTERSECTION * tris.size() * lambda(ltris.size(), rtris.size()) <
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

std::tuple<float /*cost*/, Axis /* plane axis */, float /* plane pos */,
           KDTree::TriangleIds /*left*/, KDTree::TriangleIds /*right*/>
KDTree::find_plane_and_classify(const TriangleIds& tris, const Box& box) const {
    // The box should have some surface, otherwise the surface area heuristics
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
        auto clipped_box = clip_triangle_at_aabb(tris_[id], box);
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
        return std::make_tuple(std::numeric_limits<float>::max(), Axis::X, 0,
                               TriangleIds(), TriangleIds());
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

    assert(min_ltris + (min_side == Dir::LEFT ? min_ptris : 0) == ltris.size());
    assert(min_rtris + (min_side == Dir::RIGHT ? min_ptris : 0) ==
           rtris.size());

    return std::make_tuple(min_cost, min_plane_ax, min_plane_pos,
                           std::move(ltris), std::move(rtris));
}

void KDTree::flatten() {
    static constexpr uint32_t INVALID_INDEX = 0xFFFFFFFF >> 2;

    // auto print = [this] {
    //     size_t i = 0;
    //     for (auto it = nodes_.begin(); it != nodes_.end(); ++it, ++i) {
    //         const auto& node = *it;
    //         if (node.is_inner()) {
    //             std::cerr << i << ": InnerNode("
    //                       << static_cast<int>(node.split_axis()) << ", "
    //                       << node.split_pos() << ", " << node.right() << ")"
    //                       << std::endl;
    //         } else {
    //             if (node.is_empty()) {
    //                 std::cerr << i << ": Leaf()" << std::endl;
    //             } else if (node.is_sentinel()) {
    //                 std::cerr << i << ": Leaf(" << node.first_triangle_id()
    //                           << ")" << std::endl;
    //             } else {
    //                 std::cerr << i << ": Leaf(" << node.first_triangle_id()
    //                           << ", " << node.second_triangle_id() << ")"
    //                           << std::endl;
    //             }
    //         }
    //     }
    // };

    // do DFS through nodes
    std::stack<std::pair<Node*, uint32_t /*parent index*/>> stack;
    stack.emplace(root_.get(), INVALID_INDEX);

    while (!stack.empty()) {
        const Node& node = *stack.top().first;
        uint32_t parent_index = stack.top().second;
        stack.pop();

        uint32_t node_index = nodes_.size();
        if (parent_index != INVALID_INDEX) {
            // we are in the right node => update parent's reference
            nodes_[parent_index].set_right(node_index);
        }

        if (node.is_inner()) {
            // set right to invalid, since we don't know the index yet
            nodes_.emplace_back(node.split_axis(), node.split_pos(),
                                INVALID_INDEX);

            // Push right first, since we visit left first.
            stack.emplace(node.right(), node_index);
            stack.emplace(node.left(), INVALID_INDEX);
        } else {
            bool is_odd = true;
            TriangleId first_id;
            const TriangleId* triangle_ids = node.triangle_ids();
            for (size_t i = 0; i < node.num_tris(); ++i) {
                if (is_odd) {
                    first_id = triangle_ids[i];
                } else {
                    nodes_.emplace_back(first_id, triangle_ids[i]);
                }
                is_odd = !is_odd;
            }
            // add sentinel
            if (!is_odd) {
                nodes_.emplace_back(first_id);
            } else {
                nodes_.emplace_back();
            }
        }
    }
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

/**
 *
 */
// const KDTree::OptionalId KDTreeIntersection::intersect(const Ray& ray, float&
// r,
//                                                        float& a, float& b) {
//     // A trick to make the traversal robust.
//     // Cf. [HH11], p. 5, comment about dir classification and robustness.
//     const Ray fixed_ray(ray.pos, fix_direction(ray));

//     float tenter, texit;
//     if (!intersect_ray_box(fixed_ray, tree_->box(), tenter, texit)) {
//         return OptionalId{};
//     }

//     // Note: No need to clear, since when we leave this function, the stack
//     is
//     // always empty.
//     assert(stack_.empty());
//     stack_.emplace(tree_->root_.get(), tenter, texit);

//     KDTree::Node* node;
//     OptionalId res;
//     r = std::numeric_limits<float>::max();
//     while (!stack_.empty()) {
//         std::tie(node, tenter, texit) = stack_.top();
//         stack_.pop();

//         while (node->is_inner()) {
//             int ax = static_cast<int>(node->split_axis());
//             float split_pos = node->split_pos();

//             // t at split
//             float t = (split_pos - fixed_ray.pos[ax]) * fixed_ray.invdir[ax];

//             // classify near/far with respect to t:
//             // left is near if ray.dir[ax] <= 0, else otherwise
//             KDTree::Node* near = node->left();
//             KDTree::Node* far = node->right();
//             if (fixed_ray.dir[ax] <= 0) {
//                 std::swap(near, far);
//             }

//             if (texit < t) {
//                 node = near;
//             } else if (t < tenter) {
//                 node = far;
//             } else {
//                 stack_.emplace(far, t, texit);
//                 node = near;
//                 texit = t;
//             }
//         }

//         assert(node->is_leaf());
//         float next_r, next_a, next_b;
//         auto next = intersect(node->triangle_ids(), node->num_tris(), ray,
//                               next_r, next_a, next_b);
//         if (next && next_r < r) {
//             res = next;
//             r = next_r;
//             a = next_a;
//             b = next_b;
//         }
//     }

//     return res;
// }

/**
 *
 */
const KDTree::OptionalId KDTreeIntersection::intersect(const Ray& ray, float& r,
                                                       float& a, float& b) {
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
    const auto* root = tree_->nodes_.data();
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

// const KDTree::OptionalId
// KDTreeIntersection::intersect(const KDTree::TriangleId* ids, uint32_t
// num_tris,
//                               const Ray& ray, float& min_r, float& min_s,
//                               float& min_t) {
//     min_r = std::numeric_limits<float>::max();
//     OptionalId res;
//     float r, s, t;
//     for (uint32_t i = 0; i != num_tris; ++i) {
//         const auto& tri = tree_->tris_[ids[i]];
//         bool intersects = intersect_ray_triangle(ray, tri, r, s, t);
//         if (intersects && r < min_r) {
//             min_r = r;
//             min_s = s;
//             min_t = t;
//             res = OptionalId{ids[i]};
//         }
//     }

//     return res;
// }

const KDTree::OptionalId
KDTreeIntersection::intersect(const detail::FlatNode* node, const Ray& ray,
                              float& min_r, float& min_s, float& min_t) {
    min_r = std::numeric_limits<float>::max();
    OptionalId res;

    auto intersect = [&](uint32_t triangle_id) {
        const auto& tri = tree_->tris_[triangle_id];
        float r, s, t;
        bool intersects = intersect_ray_triangle(ray, tri, r, s, t);
        if (intersects && r < min_r) {
            min_r = r;
            min_s = s;
            min_t = t;
            res = OptionalId{triangle_id};
        }
    };

    for (;; ++node) {
        assert(node->is_leaf());

        if (node->is_empty()) {
            break;
        } else {
            intersect(node->first_triangle_id());
        }

        if (node->is_sentinel()) {
            break;
        } else {
            intersect(node->second_triangle_id());
        }
    }

    return res;
}
