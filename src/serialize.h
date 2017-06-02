#pragma once

#include "geometry.h"
#include "../lib/triangle.h"

#include <cereal/types/array.hpp>
#include <cereal/types/vector.hpp>

namespace cereal {
template <class Archive> void serialize(Archive& archive, turner::Vector3f& v) {
    archive(v.x, v.y, v.z);
}

template <class Archive> void serialize(Archive& archive, turner::Point3f& p) {
    archive(p.x, p.y, p.z);
}

template <class Archive> void serialize(Archive& archive, turner::Normal3f& n) {
    archive(n.x, n.y, n.z);
}

template <class Archive> void serialize(Archive& archive, turner::Bbox3f& b) {
    archive(b.p_min, b.p_max);
}
} // namespace sereal
