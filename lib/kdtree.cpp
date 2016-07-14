#include "kdtree.h"
#include "clipping.h"

#include <stack>


namespace {

// Cf. [WH06], 5.2, Table 1
static constexpr int COST_TRAVERSAL = 15;
static constexpr int COST_INTERSECTION = 20;


// Cost function bias
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
//   ax, pos: splitting plane
//   box: AABB to split
//   num_{l,r}tris: number of triangles in the left resp. right box produces
//     by splitting `box` at `p`
//   num_ptris: number of triangles lying in the plane `p`
//
// Return:
//   cost to split the box + wether the planar triangles should be appended to
//   the lhs or rhs of the box
//
inline std::pair<float /*cost*/, Dir> surface_area_heuristics(
    Axis ax, float pos, const Box& box,
    size_t num_ltris, size_t num_rtris, size_t num_ptris)
{
    Box lbox, rbox;
    std::tie(lbox, rbox) = box.split(ax, pos);
    float area = box.surface_area();
    float larea_ratio = lbox.surface_area()/area;
    float rarea_ratio = rbox.surface_area()/area;

    auto left_planar_cost = cost(
        larea_ratio, rarea_ratio,
        num_ltris + num_ptris, num_rtris);
    auto right_planar_cost = cost(
        larea_ratio, rarea_ratio,
        num_ltris, num_ptris + num_rtris);

    if (left_planar_cost < right_planar_cost) {
        return {left_planar_cost, Dir::LEFT};
    } else {
        return {right_planar_cost, Dir::RIGHT};
    }
}

}


KDTree::KDTree(Triangles tris)
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

KDTree::Node* KDTree::build(KDTree::TriangleIds tris, const Box& box) {
    if (tris.size() == 0) {
        return nullptr;
    }

    // to few triangles -> terminate
    if (tris.size() <= 3) {
        return new Node(tris);
    }

    float min_cost;
    Axis plane_ax; float plane_pos;  // plane
    TriangleIds ltris, rtris;
    std::tie(min_cost, plane_ax, plane_pos, ltris, rtris) =
        find_plane_and_classify(tris, box);

    // clipped everything away
    if (min_cost == 0) {
        return nullptr;
    }

    // automatic termination
    // remove lambda factor from cost again, otherwise we may stuck in an
    // empty space split forever
    if (COST_INTERSECTION * tris.size()
        * lambda(ltris.size(), rtris.size()) < min_cost)
    {
        return new Node(tris);
    }

    Box lbox, rbox;
    std::tie(lbox, rbox) = box.split(plane_ax, plane_pos);

    return new Node(plane_ax, plane_pos,
        build(std::move(ltris), lbox),
        build(std::move(rtris), rbox));
}

std::tuple<
    float /*cost*/, Axis /* plane axis */, float /* plane pos */,
    KDTree::TriangleIds /*left*/, KDTree::TriangleIds /*right*/>
KDTree::find_plane_and_classify(const TriangleIds& tris, const Box& box) const
{
    float min_cost = std::numeric_limits<float>::max();
    Axis min_plane_ax; float min_plane_pos;
    Dir min_side;
    // we also store those for asserts below
    float min_ltris, min_rtris, min_ptris;

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
            events.reserve(num_tris);

            if (clipped_box.is_planar(ax)) {
                events.emplace_back(
                    Event{id, clipped_box.min[ax], clipped_box.min[ax],
                    PLANAR});
            } else {
                events.emplace_back(
                    Event{id, clipped_box.min[ax], clipped_box.max[ax],
                    STARTING});
                events.emplace_back(
                    Event{id, clipped_box.max[ax], clipped_box.min[ax],
                    ENDING});
            }
        }
    }

    // sweep
    for (auto ax : AXES) {
        auto& events = event_lists[static_cast<int>(ax)];
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
    }

    // min_cost is still infty --> terminate
    if (min_cost == std::numeric_limits<float>::max()) {
        return std::make_tuple(
            0, Axis::X, 0, TriangleIds(), TriangleIds());
    }

    // -> min_plane, min_side

    // classify

    TriangleIds ltris, rtris;
    ltris.reserve(min_ltris);
    rtris.reserve(min_rtris);

    const auto& events = event_lists[static_cast<int>(min_plane_ax)];
    for (const auto& event : events) {
        if (event.point < min_plane_pos) {
            if (event.type == ENDING ||
                event.type == PLANAR)
            {
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

    assert(min_ltris + (min_side == Dir::LEFT ? min_ptris : 0)
        == ltris.size());
    assert(min_rtris + (min_side == Dir::RIGHT ? min_ptris : 0)
        == rtris.size());

    return std::make_tuple(
        min_cost, min_plane_ax, min_plane_pos,
        std::move(ltris), std::move(rtris));
}


const KDTree::OptionalId
KDTree::intersect(const Ray& ray, float& r, float& s, float& t) const {
    float tenter, texit;
    if (!intersect_ray_box(ray, box_, tenter, texit)) {
        return OptionalId{};
    }

    std::stack<std::tuple<Node*, float /*tenter*/, float /*texit*/>> stack;
    stack.push(std::make_tuple(root_.get(), tenter, texit));

    Node* node;
    OptionalId res;
    r = std::numeric_limits<float>::max();
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
            if (ray.pos[ax] >= pos) {
                std::swap(near, far);
            }

            // Should we use a fat plane here? We would say, no!
            // t, texit and tenter are computed in exactly the same way.
            // Cf. the implementation of ray_box_intersection.
            if (t > texit || t < 0) {
                node = near;
            } else if (t < tenter) {
                node = far;
            } else {
                stack.push(std::make_tuple(far, t, texit));
                node = near;
                texit = t;
            }
        }

        if (node) {
            assert(node->is_leaf());

            float next_r, next_s, next_t;
            auto next = intersect(
                node->triangle_ids(), node->num_tris(), ray,
                next_r, next_s, next_t);
            if (next && next_r < r) {
                res = next;
                r = next_r;
                s = next_s;
                t = next_t;
            }
        }
    }

    return res;
}


const KDTree::OptionalId KDTree::intersect(
    const KDTree::TriangleId* ids, uint32_t num_tris,
    const Ray& ray, float& min_r, float& min_s, float& min_t) const
{
    min_r = std::numeric_limits<float>::max();
    OptionalId res;
    float r, s, t;
    for (uint32_t i = 0; i != num_tris; ++i) {
        const auto& tri = tris_[ids[i]];
        bool intersects = tri.intersect(ray, r, s, t);
        if (intersects && r < min_r) {
            min_r = r;
            min_s = s;
            min_t = t;
            res = OptionalId{ids[i]};
        }
    }

    return res;
}


namespace std {

size_t std::hash<KDTree::OptionalId>::operator()(
    const KDTree::OptionalId& id) const
{
    return std::hash<KDTree::TriangleId>()(id.id_);
}

} // namespace std
