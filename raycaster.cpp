#include "trace.h"
#include "lib/output.h"
#include "lib/image.h"
#include "lib/intersection.h"
#include "lib/lambertian.h"
#include "lib/triangle.h"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include <math.h>
#include <vector>
#include <iostream>

struct NodeIntersection {

    NodeIntersection() : distance(-1), mesh(nullptr), face(nullptr) {}
    NodeIntersection(float d, const aiMesh* m, const aiFace* f)
        : distance(d)
        , mesh(m)
        , face(f) {}

    bool intersects() const {
        return !(distance < 0);
    }

    bool operator<(const NodeIntersection& other) const {
        return distance < other.distance;
    }

    float distance;
    const aiMesh* mesh;
    const aiFace* face;
};

//
// Convert 2d raster coodinates into 3d cameras coordinates.
//
// We assume that the camera is trivial:
//   assert(cam.mPosition == aiVector3D(0, 0, 0));
//   assert(cam.mUp == aiVector3D(0, 1, 0));
//   assert(cam.mLookAt == aiVector3D(0, 0, -1));
//   assert(cam.mAspect != 0) {
//
// The positioning of the camera is done in its parent's node transformation
// matrix.
//
aiVector3D raster2cam(
    const aiCamera& cam, const aiVector2D& p,
    const int w, const int h)
{
    float delta_x = tan(cam.mHorizontalFOV);
    float delta_y = delta_x / cam.mAspect;
    return aiVector3D(
        -delta_x * (1 - 2 * p.x / static_cast<float>(w)),
        delta_y * (1 - 2 * p.y / static_cast<float>(h)),
        -1);
}


NodeIntersection ray_node_intersection(
    const aiRay& ray,
    const aiScene& scene, const aiNode& node)
{
    float min_r = -1;
    const aiMesh* min_mesh = nullptr;
    const aiFace* min_face = nullptr;

    for (size_t i = 0; i < node.mNumMeshes; ++i) {
        const auto& mesh = *scene.mMeshes[node.mMeshes[i]];
        const auto& T = node.mTransformation;

        for (size_t i = 0; i < mesh.mNumFaces; ++i) {
            const auto& face = mesh.mFaces[i];
            assert(face.mNumIndices == 3);

            auto V0 = T * mesh.mVertices[face.mIndices[0]];
            auto V1 = T * mesh.mVertices[face.mIndices[1]];
            auto V2 = T * mesh.mVertices[face.mIndices[2]];

            float r, s, t;
            Triangle triangle{{{V0, V1, V2}}};
            auto intersect = intersect_ray_triangle(ray, triangle, r, s, t);

            if (!intersect) {
                continue;
            }

            if (min_r < 0 || r < min_r) {
                min_r = r;
                min_mesh = &mesh;
                min_face = &face;
            }
        }
    }

    return NodeIntersection { min_r, min_mesh, min_face };
}

NodeIntersection ray_nodes_intersection(
    const aiRay& ray,
    const aiScene& scene, const std::vector<aiNode*>& nodes)
{
    // intersection
    float dist_to_triangle, s, t;
    auto triangle_id =
        triangles.intersect(aiRay{origin, dir}, dist_to_triangle, s, t);
    if (!triangle_id) {
        return conf.bg_color;
    }

    Stats::instance().num_rays += 1;
    auto res = triangles[triangle_id].diffuse;

    // The light is at camera position. The farther away an object the darker it
    // is. It's not visible beyond max visibility.
    res.a = clamp(1.f - (dist_to_triangle / conf.max_visibility), 0.f, 1.f);
    return res;
}
