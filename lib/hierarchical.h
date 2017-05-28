#pragma once

#include "algorithm.h"
#include "kdtree.h"
#include "mesh.h"
#include "output.h"
#include "progress_bar.h"
#include "radiosity.h"
#include "raster.h"
#include "types.h"

#include <iostream>
#include <sstream>
#include <stack>

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
        Quadnode* q;       // shooting node
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
        RadiosityMesh::FaceHandle face;
        std::array<RadiosityMesh::VertexHandle, 3> vs; // triangle vertex ids

        float area;

        Color rad_gather; // gathering radiosity
        Color rad_shoot;  // shooting radiosity
        Color emission;   // light emission
        Color rho;        // reflectivity (diffuse color)

        Quadnode* parent = nullptr;
        std::array<std::unique_ptr<Quadnode>, 4> children;
        std::vector<Linknode> gathering_from; // links to this node
    };

public:
    HierarchicalRadiosity(const KDTree& tree, float F_eps, float A_eps,
                          float BF_eps, size_t max_iterations)
        : tree_(&tree)
        , tree_intersection_(tree)
        , F_eps_(F_eps)
        , A_eps_(A_eps)
        , BF_eps_(BF_eps)
        , max_iterations_(max_iterations){};

    const RadiosityMesh& mesh() const { return mesh_; }

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
                const auto& p = *stack.top();
                stack.pop();

                nodes_counter += 1;

                if (!p.gathering_from.empty()) {
                    auto p_midpoint = as_point(triangle_midpoint(mesh_, p.vs));
                    auto to = cam.cam2raster(p_midpoint, image.width(),
                                             image.height());
                    for (const auto& link : p.gathering_from) {
                        auto q_midpoint =
                            as_point(triangle_midpoint(mesh_, link.q->vs));
                        auto from = cam.cam2raster(q_midpoint, image.width(),
                                                   image.height());
                        bresenham(from.x, from.y, to.x, to.y, draw_pixel);
                    }
                    links_counter += 1;
                }

                if (!p.is_leaf()) {
                    for (const auto& child : p.children) {
                        stack.push(child.get());
                    }
                }
            }
        }

        std::cerr << "Nodes " << nodes_counter << std::endl;
        std::cerr << "Links " << links_counter << std::endl;

        return image;
    }

    void compute() {
        mesh_ = build_mesh(tree_->triangles());

        // Create quad nodes
        for (size_t i = 0; i < tree_->num_triangles(); ++i) {
            nodes_.emplace_back();

            nodes_.back().root_tri_id = i;
            nodes_.back().face = RadiosityMesh::FaceHandle(i);
            nodes_.back().vs =
                triangle_vertices(mesh_, RadiosityMesh::FaceHandle(i));
            const auto& tri = (*tree_)[i];
            nodes_.back().area = tri.area();
            nodes_.back().rad_gather = Color(); // black
            nodes_.back().rad_shoot = tri.emissive;
            nodes_.back().emission = tri.emissive;
            nodes_.back().rho = tri.diffuse;
        }

        // Refine nodes
        auto progress_bar =
            ProgressBar(std::cerr, "Refine Nodes", nodes_.size());
        for (size_t n = 0; n < nodes_.size(); ++n) {
            auto& p = nodes_[n];
            for (auto& q : nodes_) {
                if (p.root_tri_id == q.root_tri_id) {
                    continue;
                }
                refine(p, q);
            }

            progress_bar.update(n + 1);
        }
        std::cerr << std::endl;

        // Solve system and refine links
        bool done = false;
        while (!done) {
            solve_system();
            done = !refine_links();
        }

        // store radiosity in mesh
        store_radiosity_at_triangles();
        std::cerr << "Triangulating T-vertices." << std::endl;
        triangulate_t_vertices();
        store_radiosity_at_vertices();
    }

    // TODO: Use mesh directly
    auto triangles() const {
        std::vector<Triangle> triangles;
        for (const auto face : mesh_.faces()) {
            const auto& vs = triangle_vertices(mesh_, face);
            // copy triangle with minimal information
            triangles.emplace_back(std::array<Point3f, 3>{
                as_point(mesh_.point(vs[0])), as_point(mesh_.point(vs[1])),
                as_point(mesh_.point(vs[2]))});
        }
        return triangles;
    }

    void store_radiosity_at_triangles() {
        auto rad = FaceRadiosityHandleProperty::createIfNotExists(
            mesh_, "face_radiosity");

        std::stack<const Quadnode*> stack;

        // dfs for each node
        for (const auto& root : nodes_) {
            stack.push(&root);
            while (!stack.empty()) {
                const auto& p = *stack.top();
                stack.pop();

                if (p.is_leaf()) {
                    rad[p.face] = p.rad_shoot;
                    rad[p.face].a = 1; // TODO
                } else {
                    for (const auto& child : p.children) {
                        stack.push(child.get());
                    }
                }
            }
        }
    }

    void store_radiosity_at_vertices() {
        auto vrad = VertexRadiosityHandleProperty::createIfNotExists(
            mesh_, "vertex_radiosity");
        FaceRadiosityHandle frad;
        auto exists = mesh_.get_property_handle(frad, "face_radiosity");
        assert(exists);
        UNUSED(exists);

        for (const auto v : mesh_.vertices()) {
            Color& avg = vrad[v];
            avg = Color(0, 0, 0, 1);

            size_t n = 0;
            for (const auto f : mesh_.vf_range(v)) {
                avg += mesh_.property(frad, f);
                n++;
            }
            avg /= n;
        }
    }

    void triangulate_t_vertices() {
        auto rad = FaceRadiosityHandleProperty::createIfNotExists(
            mesh_, "face_radiosity");

        size_t n_faces = mesh_.n_faces();
        for (size_t i = 0; i < n_faces; ++i) {
            auto face = RadiosityMesh::FaceHandle(i);
            size_t n = mesh_.n_faces();
            ::triangulate_t_vertices(mesh_, face); // no-op if nothing to do

            if (n < mesh_.n_faces()) {
                // copy radiosity
                const auto& face_rad = rad[face];
                for (; n < mesh_.n_faces(); ++n) {
                    auto new_face = RadiosityMesh::FaceHandle(n);
                    rad[new_face] = face_rad;
                }
            }
        }
    }

private:
    float estimate_form_factor(const Quadnode& p, const Quadnode& q) const {
        const auto p_midpoint = triangle_midpoint(mesh_, p.vs);
        const auto p_normal = triangle_normal(mesh_, p.vs);
        const auto q_midpoint = triangle_midpoint(mesh_, q.vs);

        const float cos_theta =
            p_normal | (q_midpoint - p_midpoint).normalize();
        assert(!std::isnan(cos_theta));
        if (cos_theta < 0) {
            return 0;
        }

        const auto& q_a = mesh_.point(q.vs[0]);
        const auto& q_b = mesh_.point(q.vs[1]);
        const auto& q_c = mesh_.point(q.vs[2]);
        const float omega_q = solid_angle(p_midpoint, q_a, q_b, q_c);

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

        auto faces = subdivide4(mesh_, p.face);
        for (size_t i = 0; i < 4; ++i) {
            auto& qnode = p.children[i];

            // create a new quanode
            qnode = std::make_unique<Quadnode>();
            qnode->parent = &p;
            qnode->rad_gather = Color();
            qnode->rad_shoot = p.rad_shoot;
            qnode->emission = p.emission;
            qnode->area = p_area_4;
            qnode->rho = p.rho;
            qnode->face = faces[i];
            qnode->vs = triangle_vertices(mesh_, faces[i]);
            qnode->root_tri_id = p.root_tri_id;
        }

        return true;
    }

    /**
     * Link p to q s.t. p gathers energy from q.
     * @param p gathering node
     * @param q shooting node
     */
    void link(Quadnode& p, Quadnode& q) {
        const auto& p_a = mesh_.point(p.vs[0]);
        const auto& p_b = mesh_.point(p.vs[1]);
        const auto& p_c = mesh_.point(p.vs[2]);
        const auto& q_a = mesh_.point(q.vs[0]);
        const auto& q_b = mesh_.point(q.vs[1]);
        const auto& q_c = mesh_.point(q.vs[2]);

        const Point3f p_pos = as_point(p_a);
        const Vector3f p_u = as_vector(p_b - p_a);
        const Vector3f p_v = as_vector(p_c - p_a);
        const Normal3f p_normal = Normal3f(normalize(cross(p_u, p_v)));
        const Point3f q_pos = as_point(q_a);
        const Vector3f q_u = as_vector(q_b - q_a);
        const Vector3f q_v = as_vector(q_c - q_a);
        const Normal3f q_normal = Normal3f(normalize(cross(q_u, q_v)));

        float F_pq =
            form_factor(tree_intersection_, p_pos, p_u, p_v, p_normal, q_pos,
                        q_u, q_v, q_normal, q.area, q.root_tri_id);
        assert(0 <= F_pq && F_pq < 1);

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
        auto progress_bar =
            ProgressBar(std::cerr, "Solving System", max_iterations_);
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
        auto progress_bar =
            ProgressBar(std::cerr, "Refining Links", nodes_.size());
        for (size_t n = 0; n < nodes_.size(); ++n) {
            refined |= refine_links(nodes_[n]);

            progress_bar.update(n + 1);
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
            for (auto& child : p.children) {
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

        auto oracle = q.rad_shoot * q.area * link_node.form_factor;
        if (oracle.r > BF_eps_ || oracle.g > BF_eps_ || oracle.b > BF_eps_) {

            float F_pq = link_node.form_factor;
            float F_qp = F_pq * p.area / q.area;

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

    // TODO: reconsider design s.t. we can avoid conversion
    // TODO: profile how much time we actually spend here!
    static Point3f as_point(const RadiosityMesh::Point& pt) {
        return Point3f{pt[0], pt[1], pt[2]};
    };
    static Vector3f as_vector(const RadiosityMesh::Point& pt) {
        return Vector3f{pt[0], pt[1], pt[2]};
    };
    static Normal3f as_normal(const RadiosityMesh::Point& pt) {
        return Normal3f{pt[0], pt[1], pt[2]};
    };

private:
    std::vector<Quadnode> nodes_;
    Triangles subdivided_tris_; // TODO: Remove
    RadiosityMesh mesh_;

    const KDTree* tree_;
    KDTreeIntersection tree_intersection_;
    float F_eps_;
    float A_eps_;
    float BF_eps_;
    int max_iterations_;
};
