#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>

namespace turner {

template <typename T> class Normal3;

/**
 * Vector in 2-dimensional space.
 */
template <typename T> class Vector2 {
public:
    Vector2() = default;
    Vector2(T x, T y) : x{x}, y{y} { assert(!contains_nan()); }

    bool contains_nan() const { return std::isnan(x) || std::isnan(y); }

    T operator[](size_t i) const {
        assert(i < 2);
        return i == 0 ? x : y;
    }

    T& operator[](size_t i) {
        assert(i < 2);
        return i == 0 ? x : y;
    }

    bool operator==(const Vector2& v) const { return x == v.x && y == v.y; }

    bool operator!=(const Vector2& v) const { return !(*this == v); }

    Vector2<T> operator+(const Vector2<T>& v) const {
        return Vector2<T>(x + v.x, y + v.y);
    }

    Vector2<T>& operator+=(const Vector2<T>& v) {
        x += v.x;
        y += v.y;
        return *this;
    }

    Vector2<T> operator-(const Vector2<T>& v) const {
        return Vector2<T>(x - v.x, y - v.y);
    }

    Vector2<T>& operator-=(const Vector2<T>& v) {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    Vector2<T> operator*(T s) const { return Vector2<T>(s * x, s * y); }

    Vector2<T>& operator*=(T s) {
        x *= s;
        y *= s;
        return *this;
    }

    Vector2<T> operator/(T s) const {
        assert(s != 0);
        return (1 / s) * *this;
    }

    Vector2<T>& operator/=(T s) const {
        assert(s != 0);
        return *this *= 1 / s;
    }

    Vector2<T> operator-() const { return Vector2<T>(-x, -y); }

    T length_squared() const { return x * x + y * y; }
    T length() const { return std::sqrt(length_squared()); }

public:
    T x = 0;
    T y = 0;
};

using Vector2f = Vector2<float>;
using Vector2i = Vector2<int>;

template <typename T> Vector2<T> operator*(T s, const Vector2<T>& v) {
    return Vector2<T>(s * v.x, s * v.y);
}

template <typename T> Vector2<T> abs(const Vector2<T>& v) {
    return Vector2<T>(std::abs(v.x), std::abs(v.y));
}

template <typename T> T dot(const Vector2<T>& v1, const Vector2<T>& v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

template <typename T> T absdot(const Vector2<T>& v1, const Vector2<T>& v2) {
    return std::abs(dot(v1, v2));
}

template <typename T> Vector2<T> normalize(const Vector2<T>& v) {
    return v / v.length();
}

template <typename T> T min_component(const Vector2<T>& v) {
    return std::min(v.x, v.y);
}

template <typename T> T max_component(const Vector2<T>& v) {
    return std::max(v.x, v.y);
}

template <typename T> int max_dimension(const Vector2<T>& v) {
    return (v.x > v.y) ? 0 : 1;
}

template <typename T>
Vector2<T> min(const Vector2<T>& p1, const Vector2<T>& p2) {
    return Vector2<T>(std::min(p1.x, p2.x), std::min(p1.y, p2.y));
}

template <typename T>
Vector2<T> max(const Vector2<T>& p1, const Vector2<T>& p2) {
    return Vector2<T>(std::max(p1.x, p2.x), std::max(p1.y, p2.y));
}

/**
 * Vector in 3-dimensional space.
 */
template <typename T> class Vector3 {
public:
    Vector3() = default;
    Vector3(T x, T y, T z) : x{x}, y{y}, z{z} { assert(!contains_nan()); }
    Vector3(const Normal3<T>& n) : x(n.x), y(n.y), z(n.z) {
        assert(!contains_nan());
    }

    bool contains_nan() const {
        return std::isnan(x) || std::isnan(y) || std::isnan(z);
    }

    T operator[](size_t i) const {
        assert(i < 3);
        return i == 0 ? x : i == 1 ? y : z;
    }

    T& operator[](size_t i) {
        assert(i < 3);
        return i == 0 ? x : i == 1 ? y : z;
    }

    bool operator==(const Vector3& v) const {
        return x == v.x && y == v.y && z == v.z;
    }

    bool operator!=(const Vector3& v) const { return !(*this == v); }

    Vector3<T> operator+(const Vector3<T>& v) const {
        return Vector3<T>(x + v.x, y + v.y, z + v.z);
    }

    Vector3<T>& operator+=(const Vector3<T>& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Vector3<T> operator-(const Vector3<T>& v) const {
        return Vector3<T>(x - v.x, y - v.y, z - v.z);
    }

    Vector3<T>& operator-=(const Vector3<T>& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Vector3<T> operator*(T s) const { return Vector3<T>(s * x, s * y, s * z); }

    Vector3<T>& operator*=(T s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    Vector3<T> operator/(T s) const {
        assert(s != 0);
        return (1 / s) * *this;
    }

    Vector3<T>& operator/=(T s) const {
        assert(s != 0);
        return *this *= 1 / s;
    }

    Vector3<T> operator-() const { return Vector3<T>(-x, -y, -z); }

    T length_squared() const { return x * x + y * y + z * z; }
    T length() const { return std::sqrt(length_squared()); }

public:
    T x = 0;
    T y = 0;
    T z = 0;
};

using Vector3f = Vector3<float>;
using Vector3i = Vector3<int>;

template <typename T> Vector3<T> operator*(T s, const Vector3<T>& v) {
    return Vector3<T>(s * v.x, s * v.y, s * v.z);
}

template <typename T> Vector3<T> abs(const Vector3<T>& v) {
    return Vector3<T>(std::abs(v.x), std::abs(v.y), std::abs(v.z));
}

template <typename T> T Dot(const Vector3<T>& v1, const Vector3<T>& v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

template <typename T> T absdot(const Vector3<T>& v1, const Vector3<T>& v2) {
    return std::abs(dot(v1, v2));
}

template <typename T>
Vector3<T> cross(const Vector3<T>& v1, const Vector3<T>& v2) {
    // use double to prevent cancelation errors
    double v1x = v1.x, v1y = v1.y, v1z = v1.z;
    double v2x = v2.x, v2y = v2.y, v2z = v2.z;
    return Vector3<T>((v1y * v2z) - (v1z * v2y), (v1z * v2x) - (v1x * v2z),
                      (v1x * v2y) - (v1y * v2x));
}

template <typename T> Vector3<T> normalize(const Vector3<T>& v) {
    return v / v.length();
}

template <typename T> T min_component(const Vector3<T>& v) {
    return std::min(v.x, std::min(v.y, v.z));
}

template <typename T> T max_component(const Vector3<T>& v) {
    return std::max(v.x, std::max(v.y, v.z));
}

template <typename T> int max_dimension(const Vector3<T>& v) {
    return (v.x > v.y) ? ((v.x > v.z) ? 0 : 2) : ((v.y > v.z) ? 1 : 2);
}

template <typename T>
Vector3<T> min(const Vector3<T>& p1, const Vector3<T>& p2) {
    return Vector3<T>(std::min(p1.x, p2.x), std::min(p1.y, p2.y),
                      std::min(p1.z, p2.z));
}

template <typename T>
Vector3<T> max(const Vector3<T>& p1, const Vector3<T>& p2) {
    return Vector3<T>(std::max(p1.x, p2.x), std::max(p1.y, p2.y),
                      std::max(p1.z, p2.z));
}

template <typename T>
Vector3<T> permute(const Vector3<T>& v, int x, int y, int z) {
    return Vector3<T>(v[x], v[y], v[z]);
}

/**
 * Point in 3-dimensional space.
 */
template <typename T> class Point3 {
public:
    Point3() = default;
    Point3(T x, T y, T z) : x{x}, y{y}, z{z} { assert(!contains_nan()); }

    template <typename U>
    explicit Point3(const Point3<U>& p)
        : x(static_cast<T>(p.x))
        , y(static_cast<T>(p.y))
        , z(static_cast<T>(p.z)) {
        assert(!contains_nan());
    }

    template <typename U> explicit operator Vector3<U>() const {
        return Vector3<U>(x, y, z);
    }

    bool contains_nan() const {
        return std::isnan(x) || std::isnan(y) || std::isnan(z);
    }

    T operator[](size_t i) const {
        assert(i < 3);
        return i == 0 ? x : i == 1 ? y : z;
    }

    T& operator[](size_t i) {
        assert(i < 3);
        return i == 0 ? x : i == 1 ? y : z;
    }

    bool operator==(const Point3<T>& v) const {
        return x == v.x && y == v.y && z == v.z;
    }

    bool operator!=(const Point3<T>& v) const { return !(*this == v); }

    Point3<T> operator+(const Vector3<T>& v) const {
        return Point3<T>(x + v.x, y + v.y, z + v.z);
    }

    Point3<T>& operator+=(const Vector3<T>& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Vector3<T> operator-(const Point3<T>& p) const {
        return Vector3<T>(x - p.x, y - p.y, z - p.z);
    }

    Point3<T> operator-(const Vector3<T>& v) const {
        return Vector3<T>(x - v.x, y - v.y, z - v.z);
    }

    Point3<T>& operator-=(const Vector3<T>& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Point3<T> operator*(T s) const { return Point3<T>(s * x, s * y, s * z); }

    Point3<T>& operator*=(T s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    Point3<T> operator/(T s) const {
        assert(s != 0);
        return (1 / s) * *this;
    }

    Point3<T>& operator/=(T s) const {
        assert(s != 0);
        return *this *= 1 / s;
    }

public:
    T x = 0;
    T y = 0;
    T z = 0;
};

using Point3f = Point3<float>;
using Point3i = Point3<int>;

template <typename T> float distance(const Point3<T>& p1, const Point3<T>& p2) {
    return (p1 - p2).length();
}

template <typename T>
float distance_squared(const Point3<T>& p1, const Point3<T>& p2) {
    return (p1 - p2).length_squared();
}

/**
 * Linearly interpolate between two points.
 */
template <typename T>
Point3<T> lerp(float t, const Point3<T>& p0, const Point3<T>& p1) {
    return (1 - t) * p0 + t * p1;
}

template <typename T> Point3<T> min(const Point3<T>& p1, const Point3<T>& p2) {
    return Point3<T>(std::min(p1.x, p2.x), std::min(p1.y, p2.y),
                     std::min(p1.z, p2.z));
}

template <typename T> Point3<T> max(const Point3<T>& p1, const Point3<T>& p2) {
    return Point3<T>(std::max(p1.x, p2.x), std::max(p1.y, p2.y),
                     std::max(p1.z, p2.z));
}

template <typename T> Point3<T> floor(const Point3<T>& p) {
    return Point3<T>(std::floor(p.x), std::floor(p.y), std::floor(p.z));
}

template <typename T> Point3<T> ceil(const Point3<T>& p) {
    return Point3<T>(std::ceil(p.x), std::ceil(p.y), std::ceil(p.z));
}

template <typename T> Point3<T> abs(const Point3<T>& p) {
    return Point3<T>(std::abs(p.x), std::abs(p.y), std::abs(p.z));
}

template <typename T>
Point3<T> permute(const Point3<T>& p, int x, int y, int z) {
    return Point3<T>(p[x], p[y], p[z]);
}

/**
 * Point in 3-dimensional space.
 */
template <typename T> class Normal3 {
public:
    Normal3() = default;
    Normal3(T x, T y, T z) : x{x}, y{y}, z{z} { assert(!contains_nan()); }

    explicit Normal3(const Vector3f<T>& v) : x(v.x), y(v.y), z(v.z) {
        assert(!contains_nan());
    }

    template <typename U>
    explicit Point3(const Point3<U>& p)
        : x(static_cast<T>(p.x))
        , y(static_cast<T>(p.y))
        , z(static_cast<T>(p.z)) {
        assert(!contains_nan());
    }

    template <typename U> explicit operator Vector3<U>() const {
        return Vector3<U>(x, y, z);
    }

    bool contains_nan() const {
        return std::isnan(x) || std::isnan(y) || std::isnan(z);
    }

    T operator[](size_t i) const {
        assert(i < 3);
        return i == 0 ? x : i == 1 ? y : z;
    }

    T& operator[](size_t i) {
        assert(i < 3);
        return i == 0 ? x : i == 1 ? y : z;
    }

    bool operator==(const Point3<T>& v) const {
        return x == v.x && y == v.y && z == v.z;
    }

    bool operator!=(const Point3<T>& v) const { return !(*this == v); }

    Point3<T> operator+(const Vector3<T>& v) const {
        return Point3<T>(x + v.x, y + v.y, z + v.z);
    }

    Point3<T>& operator+=(const Vector3<T>& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Vector3<T> operator-(const Point3<T>& p) const {
        return Vector3<T>(x - p.x, y - p.y, z - p.z);
    }

    Point3<T> operator-(const Vector3<T>& v) const {
        return Vector3<T>(x - v.x, y - v.y, z - v.z);
    }

    Point3<T>& operator-=(const Vector3<T>& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Point3<T> operator*(T s) const { return Point3<T>(s * x, s * y, s * z); }

    Point3<T>& operator*=(T s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    Point3<T> operator/(T s) const {
        assert(s != 0);
        return (1 / s) * *this;
    }

    Point3<T>& operator/=(T s) const {
        assert(s != 0);
        return *this *= 1 / s;
    }

public:
    T x = 0;
    T y = 0;
    T z = 0;
};

using Normal3f = Normal3<float>;

template <typename T> T dot(const Normal3<T>& n1, const Normal3<T>& n2) {
    return n1.x * n2.x + n1.y * n2.y + n1.z * n2.z;
}

template <typename T> T absdot(const Normal3<T>& n1, const Normal3<T>& n2) {
    return std::abs(dot(n1, n2));
}

/**
 * Infinite ray in 3d space.
 */
class Ray {
public:
    Ray() = default;
    Ray(const Point3f& o, const Vector3f& d,
        float t_max = std::numeric_limits<float>::max())
        : o(o), d(d), t_max(t_max) {}

    Point3f operator()(float t) const { return o + d * t; }

public:
    Point3f o;
    Vector3f d;
    float t_max = std::numeric_limits<float>::max(); // max ray extension
};

template <typename T> class Bbox3 {
public:
    Bbox3()
        : p_min(std::numeric_limits<T>::lowest(),
                std::numeric_limits<T>::lowest(),
                std::numeric_limits<T>::lowest())
        , p_max(std::numeric_limits<T>::max(), std::numeric_limits<T>::max(),
                std::numeric_limits<T>::max()) {}
    Bbox3(const Point3<T>& p) : p_min(p), p_max(p) {}
    Bounds3(const Point3<T>& p1, const Point3<T>& p2)
        : pMin(min(p1, p2)), pMax(max(p1, p2)) {}

    const Point3<T>& operator[](size_t i) const {
        assert(i < 2);
        return i == 0 ? p_min : p_max;
    }

    Point3<T>& operator[](size_t i) {
        assert(i < 2);
        return i == 0 ? p_min : p_max;
    }

    Point3<T> corner(size_t corner) const {
        return {(*this)[(corner & 1)].x, (*this)[(corner & 2) ? 1 : 0].y,
                (*this)[(corner & 4) ? 1 : 0].z};
    }

    Vector3<T> diagonal() const { return p_max - p_min; }

    T SurfaceArea() const {
        Vector3<T> d = diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    T Volume() const {
        Vector3<T> d = diagonal();
        return d.x * d.y * d.z;
    }

    Point3<T> lerp(const Point3f& t) const {
        return {lerp(t.x, p_min.x, p_max.x), lerp(t.y, p_min.y, p_max.y),
                lerp(t.z, p_min.z, p_max.z)};

    public:
        Point3<T> p_min;
        Point3<T> p_max;
    };

    using Bbox3f = Bbox3<float>;
    using Bbox3i = Bbox3<int>;

    template <typename T>
    Bbox3<T> union(const Bbox3<T>& b, const Point3<T>& p) {
        return {min(b.p_min, p), max(b.p_max, p)};
    }

    template <typename T>
    Bbox3<T> union(const Bbox3<T>& b1, const Bbox3<T>& b2) {
        return {min(b1.p_min, b2.p_min), max(b1.p_max, b2.p_max)};
    }

    template <typename T>
    Bbox3<T> intersect(const Bbox3<T>& b1, const Bbox3<T>& b2) {
        return {max(b1.p_min, b2.p_min), min(b1.p_max, b2.p_max)};
    }

    template <typename T>
    bool overlaps(const Bbox3<T>& b1, const Bbox3<T>& b2) {
        bool x = (b1.p_max.x >= b2.p_min.x) && (b1.p_min.x <= b2.p_max.x);
        bool y = (b1.p_max.y >= b2.p_min.y) && (b1.p_min.y <= b2.p_max.y);
        bool z = (b1.p_max.z >= b2.p_min.z) && (b1.p_min.z <= b2.p_max.z);
        return x && y && z;
    }

    template <typename T> bool inside(const Point3<T>& p, const Bbox3<T>& b) {
        return (p.x >= b.p_min.x && p.x <= b.p_max.x && p.y >= b.p_min.y &&
                p.y <= b.p_max.y && p.z >= b.p_min.z && p.z <= b.p_max.z);
    }

} // namespace turner
