#pragma once

#include "geometry.h"

#include <functional>
#include <utility>

inline void hash_combine(size_t&) {}

template <typename T, typename... Ts>
void hash_combine(size_t& seed, const T& v, Ts... rest) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    hash_combine(seed, rest...);
}

namespace std {
template <> struct hash<turner::Vector3f> {
    size_t operator()(const turner::Vector3f& v) const {
        size_t seed = 0;
        hash_combine(seed, v.x, v.y, v.z);
        return seed;
    }
};

template <> struct hash<turner::Point3f> {
    size_t operator()(const turner::Point3f& p) const {
        size_t seed = 0;
        hash_combine(seed, p.x, p.y, p.z);
        return seed;
    }
};

template <> struct hash<turner::Normal3f> {
    size_t operator()(const turner::Normal3f& n) const {
        size_t seed = 0;
        hash_combine(seed, n.x, n.y, n.z);
        return seed;
    }
};

template <typename X, typename Y> struct hash<std::pair<X, Y>> {
    size_t operator()(const std::pair<X, Y>& val) const {
        size_t seed = 0;
        hash_combine(seed, val.first, val.second);
        return seed;
    }
};
}; // namespace std
