#pragma once

#include "types.h"

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
template <> struct hash<Vec> {
    size_t operator()(const Vec& v) const {
        size_t seed = 0;
        hash_combine(seed, v.x, v.y, v.z);
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

template <typename Iter1, typename Iter2>
class ZipIterator : public std::random_access_iterator_tag {
    using T1 = typename std::iterator_traits<Iter1>::value_type;
    using T2 = typename std::iterator_traits<Iter2>::value_type;

    struct ProxyHolder {
        using T = const std::pair<const T1&, const T2&>;
        ProxyHolder(T val) : val(val) {}
        T* operator->() const { return &val; };
        T val;
    };

public:
    ZipIterator(Iter1 it1, Iter2 it2) : it1_(it1), it2_(it2) {}

    ZipIterator& operator+=(int n) {
        advance(n);
        return *this;
    }

    friend ZipIterator operator+(ZipIterator it, int n) {
        ZipIterator temp = it;
        temp.advance(n);
        return temp;
    }

    friend ZipIterator operator+(int n, ZipIterator it) { return it + n; }

    ZipIterator& operator-=(int n) {
        advance(-n);
        return *this;
    }

    friend ZipIterator operator-(ZipIterator it, int n) {
        ZipIterator temp = it;
        temp.advance(-n);
        return temp;
    }

    ZipIterator operator++() {
        advance();
        return *this;
    }

    ZipIterator operator++(int) {
        ZipIterator previous = *this;
        advance();
        return previous;
    }

    ZipIterator operator--() {
        advance(-1);
        return *this;
    }

    ZipIterator operator--(int) {
        ZipIterator previous = *this;
        advance(-1);
        return previous;
    }

    friend size_t operator-(ZipIterator a, ZipIterator b) {
        size_t n = a.it1_ - b.it1_;
        size_t m = a.it2_ - b.it2_;
        assert(n == m);
        return n;
    }

    bool operator==(const ZipIterator& other) const {
        return it1_ == other.it1_ && it2_ == other.it2_;
    }

    bool operator!=(const ZipIterator& other) const {
        return !(*this == other);
    }

    const std::pair<const T1&, const T2&> operator*() const {
        return std::make_pair(*it1_, *it2_);
    }

    const ProxyHolder operator->() const { return ProxyHolder(**this); }

private:
    void advance(int n = 1) {
        it1_ += n;
        it2_ += n;
    }

private:
    Iter1 it1_;
    Iter2 it2_;
};

template <typename Iter1, typename Iter2>
ZipIterator<Iter1, Iter2> make_zip_iterator(Iter1 it1, Iter2 it2) {
    return ZipIterator<Iter1, Iter2>(it1, it2);
}

namespace std {
template <typename Iter1, typename Iter2>
size_t distance(ZipIterator<Iter1, Iter2> it1, ZipIterator<Iter1, Iter2> it2) {
    return it2 - it1;
}
} // namespace std
