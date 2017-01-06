#pragma once

#include "output.h"
#include "progress_bar.h"
#include "radiosity.h"
#include "raster.h"
#include "solid_angle.h"
#include "types.h"

#include <iostream>
#include <sstream>
#include <stack>

inline std::array<Triangle, 4> subdivide4(const Triangle& tri) {
    const auto& a = tri.vertices[0];
    const auto& b = tri.vertices[1];
    const auto& c = tri.vertices[2];

    const auto& na = tri.normals[0];
    const auto& nb = tri.normals[1];
    const auto& nc = tri.normals[2];

    auto ma = (b + c) / 2.f;
    auto mb = (a + c) / 2.f;
    auto mc = (a + b) / 2.f;

    auto nma = tri.interpolate_normal(0.f, 0.5f, 0.5f);
    auto nmb = tri.interpolate_normal(0.5f, 0.f, 0.5f);
    auto nmc = tri.interpolate_normal(0.5f, 0.5f, 0.f);

    auto copy_tri = [&tri](const Vec& a, const Vec& b, const Vec& c,
                           const Vec& na, const Vec& nb, const Vec& nc) {
        return Triangle{{a, b, c},       {na, nb, nc}, tri.ambient,
                        tri.diffuse,     tri.emissive, tri.reflective,
                        tri.reflectivity};
    };

    return {
        copy_tri(a, mc, mb, na, nmc, nmb), copy_tri(mc, b, ma, nmc, nb, nma),
        copy_tri(mb, ma, c, nmb, nma, nc), copy_tri(mb, mc, ma, nmb, nmc, nma)};
};

// https://graphics.stanford.edu/papers/rad/
class HierarchicalRadiosity {
    using TriangleId = KDTree::TriangleId;

    struct Quadnode;

    /**
     * Links a node p to a node q.
     *
     * The node p is always the owner of the link. A link from p to q means that
     * p gathers radiosity from q, In particular, `form_factor` is F_pq.
     */
    struct Linknode {
        Quadnode* q; // shooting node
        float form_factor; // form factor F_pq, where p is the owner node
    };

    struct Quadnode {
        bool operator==(const Quadnode&) const { return true; }
        bool is_leaf() const {
            // due to full subdivision:
            // if some child is none => all children are none
            return children[0] == nullptr;
        }

        TriangleId root_tri_id; // original parent triangle from scene
        TriangleId tri_id;      // underlying triangle (if this quadnode
                                // represents a root node, then this is equal
                                // to root_tri_id)
        float area;

        Color rad_gather; // gathering radiosity
        Color rad_shoot;  // shooting radiosity
        // TODO: Remove, since the data is already in the triangle
        Color emission; // light emission
        Color rho;      // reflectivity (diffuse color)

        Quadnode* parent = nullptr;
        std::array<std::unique_ptr<Quadnode>, 4> children;
        std::vector<Linknode> gathering_from; // links to this node
    };

public:
    HierarchicalRadiosity(const KDTree& tree, float F_eps, float A_eps,
            size_t max_iterations, float BF_eps)
        : tree_(&tree), F_eps_(F_eps), A_eps_(A_eps), BF_eps_(BF_eps),
          max_iterations_(max_iterations){};

    Image visualize_links(const Camera& cam, Image&& image) const {
        auto draw_pixel = [&image](int x, int y) {
            if (0 <= x && static_cast<size_t>(x) < image.width() && 0 <= y &&
                static_cast<size_t>(y) < image.height()) {
                image(x, y) = Color();
            }
        };

        size_t nodes_counter = 0;
        size_t links_counter = 0;

        std::stack<const Quadnode*> stack;
        for (const auto& root : nodes_) {
            stack.push(&root);
            while (!stack.empty()) {
                auto p = stack.top();
                stack.pop();

                nodes_counter += 1;

                if (!p->gathering_from.empty()) {
                    auto to = cam.cam2raster(get_triangle(*p).midpoint(),
                                             image.width(), image.height());
                    for (const auto& link : p->gathering_from) {
                        auto from =
                            cam.cam2raster(get_triangle(*link.q).midpoint(),
                                           image.width(), image.height());
                        bresenham(from.x, from.y, to.x, to.y, draw_pixel);
                    }
                    links_counter += 1;
                }

                if (!p->is_leaf()) {
                    for (const auto& child : p->children) {
                        stack.push(child.get());
                    }
                }
            }
        }

        std::cerr << "Nodes " << nodes_counter << std::endl;
        std::cerr << "Links " << links_counter << std::endl;

        return image;
    }

    std::string get_id(const Quadnode* p) {
        std::stringstream os;
        if (!is_root(*p)) {
            os << static_cast<size_t>(p->tri_id) << " (" << p->root_tri_id
               << ")";
        } else {
            os << p->root_tri_id;
        }
        return os.str();
    }

    auto compute() {
        // Create quad nodes
        for (size_t i = 0; i < tree_->num_triangles(); ++i) {
            nodes_.emplace_back();

            nodes_.back().root_tri_id = i;
            nodes_.back().tri_id = i;
            const auto& tri = (*tree_)[i];
            nodes_.back().area = tri.area();
            nodes_.back().rad_gather = Color(); // black
            nodes_.back().rad_shoot = tri.emissive;
            nodes_.back().emission = tri.emissive;
            nodes_.back().rho = tri.diffuse;
        }

        // Refine nodes
        auto progress_bar = ProgressBar(std::cerr, "Refine Nodes", nodes_.size());
        for (size_t n = 0; n < nodes_.size(); ++n) {
            auto&p = nodes_[n];
            for (auto& q : nodes_) {
                if (p.root_tri_id == q.root_tri_id) {
                    continue;
                }
                refine(p, q);
            }

            progress_bar.update(n+1);
        }
        std::cerr << std::endl;

        // Solve system and refine links
        bool done = false;
        while (!done) {
            solve_system();

            done = !refine_links();
        }

        // Return leaves
        std::pair<std::vector<Triangle>, std::vector<Color /*rad*/>> out;
        std::stack<const Quadnode*> stack;

        // dfs for each node
        for (const auto& root : nodes_) {
            stack.push(&root);

            std::cerr << "Root " << root.root_tri_id << std::endl;

            while (!stack.empty()) {
                auto p = stack.top();
                stack.pop();

                std::cerr << "Quadnode " << get_id(p) << std::endl;
                std::cerr << "  Triangle: A = " << get_triangle(*p).area()
                          << std::endl;
                std::cerr << "  " << p->gathering_from.size() << " Links "
                          << std::endl;
                for (const auto& link : p->gathering_from) {
                    std::cerr << "    <- " << get_id(link.q)
                              << " with F = " << link.form_factor << std::endl;
                }

                std::cerr << get_id(p) << " : " << p->emission << std::endl;

                if (p->is_leaf()) {
                    out.first.emplace_back(get_triangle(*p));
                    out.second.emplace_back(p->rad_shoot);
                    out.second.back().a = 1; // TODO
                } else {
                    for (const auto& child : p->children) {
                        stack.push(child.get());
                    }
                }
            }
        }
        return out;
    }

private:

    TriangleId next_triangle_id() const {
        return tree_->num_triangles() + subdivided_tris_.size();
    }

    bool is_root(const Quadnode& p) const {
        return p.tri_id < tree_->num_triangles();
    }

    const Triangle& get_triangle(const Quadnode& p) const {
        if (is_root(p)) {
            return (*tree_)[p.root_tri_id];
        }
        return subdivided_tris_[p.tri_id - tree_->num_triangles()];
    }

    float estimate_form_factor(const Quadnode& p, const Quadnode& q) const {
        const Triangle& tri_p = get_triangle(p);
        const Triangle& tri_q = get_triangle(q);

        const Vec p_midpoint = tri_p.midpoint();
        const Vec q_midpoint = tri_q.midpoint();

        const float cos_theta =
            tri_p.normal * (q_midpoint - p_midpoint).Normalize();
        if (cos_theta < 0) {
            return 0;
        }

        const float omega_q = solid_angle(p_midpoint, tri_q);
        const float factor = cos_theta * omega_q / M_PI;
        return factor;
    }

    bool subdivide(Quadnode& p) {
        if (!p.is_leaf()) {
            return true;
        }

        float p_area_4 = p.area / 4;
        if (p_area_4 < A_eps_) {
            return false;
        }

        const auto& child_tris = subdivide4(get_triangle(p));
        for (size_t i = 0; i < 4; ++i) {
            const auto& tri = child_tris[i];
            auto& qnode = p.children[i];

            // create a new quanode
            qnode = std::make_unique<Quadnode>();
            qnode->parent = &p;
            qnode->rad_gather = Color();
            qnode->rad_shoot = p.rad_shoot;
            qnode->emission = p.emission;
            qnode->area = p_area_4;
            qnode->rho = p.rho;
            qnode->tri_id = next_triangle_id();
            qnode->root_tri_id = p.root_tri_id;

            // add a new subdivided traingle
            subdivided_tris_.emplace_back(tri);
        }

        return true;
    }

    /**
     * Link p to q s.t. p gathers energy from q.
     * @param p gathering node
     * @param q shooting node
     */
    void link(Quadnode& p, Quadnode& q) {
        const Triangle& tri_p = get_triangle(p);
        const Triangle& tri_q = get_triangle(q);
        float F_pq = form_factor(*tree_, tri_p, tri_q, q.root_tri_id);
        // assert(0 <= F_pq && F_pq < 1); // TODO: not true now

        p.gathering_from.emplace_back();
        auto& link = p.gathering_from.back();
        link.q = &q;
        link.form_factor = F_pq;
    };

    void refine(Quadnode& p, Quadnode& q) {

        std::stack<std::pair<Quadnode*, Quadnode*>> node_stack;
        node_stack.push({&p, &q});
        while (!node_stack.empty()) {

            auto pq = node_stack.top();
            node_stack.pop();

            auto& p = *pq.first;
            auto& q = *pq.second;

            float F_pq = estimate_form_factor(p, q);
            float F_qp = estimate_form_factor(q, p);
            if (F_pq < F_eps_ && F_qp < F_eps_) {
                link(p, q);
                continue;
            }

            if (F_qp < F_pq) {
                if (subdivide(q)) {
                    for (auto& child : q.children) {
                        node_stack.push({&p, child.get()});
                    }
                    continue;
                }
            } else {
                if (subdivide(p)) {
                    for (auto& child : p.children) {
                        node_stack.push({child.get(), &q});
                    }
                    continue;
                }
            }

            link(p, q);
        }
    }

    void solve_system() {
        size_t iteration = max_iterations_;
        auto progress_bar = ProgressBar(std::cerr, "Solving System", max_iterations_);
        while (iteration--) // TODO: need a better convergence criteria
        {
            for (auto& p : nodes_) {
                gather_radiosity(p);
            }
            for (auto& p : nodes_) {
                push_pull_radiosity(p, Color());
            }

            progress_bar.update(max_iterations_ - iteration);
        }
        std::cerr << std::endl;
    }

    /**
     * Refine all links in all nodes.
     * @return true if at least one link has been refined.
     */
    bool refine_links() {
        bool refined = false;
        auto progress_bar = ProgressBar(std::cerr, "Refining Links", nodes_.size());
        for (size_t n = 0; n < nodes_.size(); ++n) {
            refined |= refine_links(nodes_[n]);

            progress_bar.update(n+1);
        }
        std::cerr << std::endl;

        return refined;
    }

    /**
     * Refine all links in node p.
     * @return true if at least one link has been refined.
     */
    bool refine_links(Quadnode& p) {
        bool refined = false;

        // Process all child nodes first.
        if (!p.is_leaf()) {
            for (auto& child: p.children) {
                refined |= refine_links(*child.get());
            }
        }

        // Post-order: Process links.
        // Remove link if we refined. Note: new links might be added that's why
        // we keep track of the old size.
        std::vector<Linknode>& links = p.gathering_from;
        size_t size = links.size();
        size_t i = 0;
        while (i < size) {
            if (refine_link(p, links[i])) {
                links.erase(links.begin() + i);
                --size;

                refined |= true;
            } else {
                ++i;
            }
        }

        return refined;
    }

    /**
     * Refine link of receiver node p.
     *
     * @param p receiver node
     * @param link link between shooter and receiver node
     * @return true if a link has been refined.
     */
    bool refine_link(Quadnode& p, Linknode& link_node) {

        // Shooter node p
        Quadnode& q = *link_node.q;

        const Triangle& tri_p = get_triangle(p);
        const Triangle& tri_q = get_triangle(q);

        auto oracle = q.rad_shoot * tri_q.area() * link_node.form_factor;
        if (oracle.r > BF_eps_ || oracle.g > BF_eps_ || oracle.b > BF_eps_) {

            float F_pq = link_node.form_factor;
            float F_qp = F_pq * tri_p.area() / tri_q.area();

            // Decide which side to subdivide. See refine()
            if (F_pq < F_qp) {
                if (subdivide(p)) {
                    // We've subdivided reciever node p. So all children of p
                    // should gather from q now.
                    for (auto& child : p.children) {
                        link(*child.get(), q);
                    }

                    return true;
                }
            } else {
                if (subdivide(q)) {
                    // We've subdivided shooter node q. So receiver node p
                    // should gather from all children of q now.
                    for (auto& child : q.children) {
                        link(p, *child.get());
                    }

                    return true;
                }
            }
        }

        return false;
    }

    void gather_radiosity(Quadnode& in) {
        std::stack<Quadnode*> node_stack;
        node_stack.push(&in);
        while (!node_stack.empty()) {
            auto& p = *node_stack.top();
            node_stack.pop();

            p.rad_gather = Color();
            for (const auto& link : p.gathering_from) {
                p.rad_gather += link.form_factor * link.q->rad_shoot;
            }
            p.rad_gather = p.rho * p.rad_gather;

            if (p.is_leaf()) {
                continue;
            }

            for (auto& child : p.children) {
                node_stack.push(child.get());
            }
        }
    }

    Color push_pull_radiosity(Quadnode& p, const Color& rad_down) {
        if (p.is_leaf()) {
            p.rad_shoot = p.emission + p.rad_gather + rad_down;
        } else {
            Color rad_up = Color();
            for (auto& child : p.children) {
                rad_up += push_pull_radiosity(*child, p.rad_gather + rad_down);
            }
            p.rad_shoot = rad_up / 4.f;
        }
        return p.rad_shoot;
    }

private:
    std::vector<Quadnode> nodes_;
    Triangles subdivided_tris_;

    const KDTree* tree_;
    float F_eps_;
    float A_eps_;
    float BF_eps_;
    int max_iterations_;
};
