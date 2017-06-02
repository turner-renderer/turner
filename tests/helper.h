#include "../lib/triangle.h"
#include "../lib/types.h"

#include <math.h>
#include <random>

// Construct a triangle with trivial normals and colors.
Triangle test_triangle(Point3f a, Point3f b, Point3f c) {
    return Triangle({a, b, c});
}

// Construct a triangle with trivial colors.
Triangle test_triangle(Point3f a, Point3f b, Point3f c, Normal3f na,
                       Normal3f nb, Normal3f nc) {
    return Triangle({a, b, c}, {na, nb, nc}, Color(), Color(), Color(), Color(),
                    0);
}

Vector3f random_vec() {
    static std::default_random_engine gen(0);
    static std::uniform_real_distribution<float> rnd(-10.f, 10.f);
    return {rnd(gen), rnd(gen), rnd(gen)};
};

Point3f random_point() {
    static std::default_random_engine gen(1);
    static std::uniform_real_distribution<float> rnd(-10.f, 10.f);
    return {rnd(gen), rnd(gen), rnd(gen)};
};

Normal3f random_normal() {
    static std::default_random_engine gen(2);
    static std::uniform_real_distribution<float> rnd(-10.f, 10.f);
    return {rnd(gen), rnd(gen), rnd(gen)};
};

// Construct a random vector lying on the unit sphere in the plane ax = pos.
Point3f random_vec_on_unit_sphere(Axis3 ax, float pos) {
    static std::default_random_engine gen(3);
    static std::uniform_real_distribution<float> rnd(0, 2 * M_PI);

    auto phi = rnd(gen);
    auto x = cos(phi);
    auto y = sin(phi);

    Point3f p;
    size_t ax_num = static_cast<size_t>(ax);
    p[AXES3[(ax_num + 0) % 3]] = pos;
    p[AXES3[(ax_num + 1) % 3]] = x;
    p[AXES3[(ax_num + 2) % 3]] = y;
    return p;
}

Triangle random_triangle() {
    return test_triangle(random_point(), random_point(), random_point());
}

// Construct a random triangle with vertices lying on the unit sphere in the
// plane ax = pos.
Triangle random_triangle_on_unit_sphere(Axis3 ax, float pos) {
    return test_triangle(random_vec_on_unit_sphere(ax, pos),
                         random_vec_on_unit_sphere(ax, pos),
                         random_vec_on_unit_sphere(ax, pos));
}

// Construct a random regular triangle with vertices lying on the unit sphere
// in the plane ax = pos. In particular, the 0-point lies in the triangle.
Triangle random_regular_triangle_on_unit_sphere(Axis3 ax, float pos) {
    static std::default_random_engine gen(0);
    static std::uniform_real_distribution<float> rnd(0, 2 * M_PI);

    auto phi = rnd(gen);
    Point3f vertices[3];
    for (size_t i = 0; i < 3; ++i) {
        auto x = cos(phi + 2.f * M_PI / 3 * i);
        auto y = sin(phi + 2.f * M_PI / 3 * i);

        size_t ax_num = static_cast<size_t>(ax);
        vertices[i][AXES3[ax_num]] = pos;
        vertices[i][AXES3[(ax_num + 1) % 3]] = x;
        vertices[i][AXES3[(ax_num + 2) % 3]] = y;
    }

    return test_triangle(vertices[0], vertices[1], vertices[2]);
}

// We need this operator only for tests.
bool operator==(const Triangle& tria, const Triangle& trib) {
    return tria.vertices[0] == trib.vertices[0] &&
           tria.vertices[1] == trib.vertices[1] &&
           tria.vertices[2] == trib.vertices[2];
}
