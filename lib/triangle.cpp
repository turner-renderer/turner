#include "triangle.h"
#include "types.h"
#include "intersection.h"


bool Triangle::intersect(const Ray& ray, float& r, float& s, float& t) const {
    return intersect_ray_triangle(ray, *this, r, s, t);
}

bool Triangle::intersect(const Box& box) const {
    return intersect_triangle_box(*this, box);
}
