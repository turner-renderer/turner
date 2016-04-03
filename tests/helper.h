#include "../lib/types.h"
#include "../lib/triangle.h"
#include <random>


// Construct a triangle with trivial normals and colors.
Triangle test_triangle(Vec a, Vec b, Vec c) {
    return Triangle({{a, b, c}}, {{Vec{}, Vec{}, Vec{}}}, {}, {}, {}, 0.f);
}

Vec random_vec() {
    static std::default_random_engine gen(0);
    static std::uniform_real_distribution<float> rnd(-10.f, 10.f);
    return {rnd(gen), rnd(gen), rnd(gen)};
};

Triangle random_triangle() {
    return test_triangle(random_vec(), random_vec(), random_vec());
}

// We need this operator only for tests.
bool operator==(const Box& b1, const Box& b2) {
    return b1.min == b2.min && b2.max == b2.max;
}

// We need this operator only for tests.
bool operator==(const Triangle& tria, const Triangle& trib) {
    return
        tria.vertices[0] == trib.vertices[0] &&
        tria.vertices[1] == trib.vertices[1] &&
        tria.vertices[2] == trib.vertices[2];
}
