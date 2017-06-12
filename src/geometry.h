#pragma once

#include "turner.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <string>

namespace turner {

/**
 * Axes in 2-dimensional space.
 */
enum class Axis2 : char { X = 0, Y = 1 };
static constexpr std::array<Axis2, 2> AXES2 = {{Axis2::X, Axis2::Y}};

/**
 * Vector in 2-dimensional space.
 */
template <typename T> class Vector2 {
public:
    Vector2() = default;
    Vector2(T x, T y) : x{x}, y{y} { assert(!contains_nan()); }

    template <typename U>
    explicit Vector2(const Vector2<U>& v)
        : x{static_cast<T>(v.x)}, y{static_cast<T>(v.y)} {
        assert(!contains_nan());
    }

    template <typename U> Vector2<U> as() const { return Vector2<U>(*this); }

    bool contains_nan() const { return std::isnan(x) || std::isnan(y); }

    T operator[](Axis2 ax) const { return ax == Axis2::X ? x : y; }

    T& operator[](Axis2 ax) { return ax == Axis2::X ? x : y; }

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
        return {x + v.x, y + v.y};
    }

    Vector2<T>& operator+=(const Vector2<T>& v) {
        x += v.x;
        y += v.y;
        return *this;
    }

    Vector2<T> operator-(const Vector2<T>& v) const {
        return {x - v.x, y - v.y};
    }

    Vector2<T>& operator-=(const Vector2<T>& v) {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    Vector2<T> operator*(T s) const { return {s * x, s * y}; }

    Vector2<T>& operator*=(T s) {
        x *= s;
        y *= s;
        return *this;
    }

    Vector2<T> operator/(T s) const {
        assert(s != 0);
        return (1 / s) * *this;
    }

    Vector2<T>& operator/=(T s) {
        assert(s != 0);
        return *this *= 1 / s;
    }

    Vector2<T> operator-() const { return {-x, -y}; }

    T length_squared() const { return x * x + y * y; }
    T length() const { return std::sqrt(length_squared()); }

    std::string to_string() const {
        return "[" + std::to_string(x) + ", " + std::to_string(y) + "]";
    }

public:
    T x = 0;
    T y = 0;
};

using Vector2f = Vector2<float>;
using Vector2i = Vector2<int>;

template <typename T> Vector2<T> operator*(T s, const Vector2<T>& v) {
    return {s * v.x, s * v.y};
}

template <typename T> Vector2<T> abs(const Vector2<T>& v) {
    return {std::abs(v.x), std::abs(v.y)};
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
    return {std::min(p1.x, p2.x), std::min(p1.y, p2.y)};
}

template <typename T>
Vector2<T> max(const Vector2<T>& p1, const Vector2<T>& p2) {
    return {std::max(p1.x, p2.x), std::max(p1.y, p2.y)};
}

/**
 * Axes in 2-dimensional space.
 */
enum class Axis3 : char { X = 0, Y = 1, Z = 2 };
constexpr std::array<Axis3, 3> AXES3 = {{Axis3::X, Axis3::Y, Axis3::Z}};

/**
 * Vector in 3-dimensional space.
 */

// Forward declaration for conversion operators and constructors
template <typename T> class Normal3;

template <typename T> class Vector3 {
public:
    Vector3() = default;
    Vector3(T x, T y, T z) : x{x}, y{y}, z{z} { assert(!contains_nan()); }

    template <typename U>
    explicit Vector3(const Vector3<U>& v)
        : x{static_cast<T>(v.x)}
        , y{static_cast<T>(v.y)}
        , z{static_cast<T>(v.z)} {
        assert(!contains_nan());
    }

    template <typename U> Vector3<U> as() const { return Vector3<U>(*this); }

    explicit Vector3(const Normal3<T>& n) : x(n.x), y(n.y), z(n.z) {
        assert(!contains_nan());
    }

    bool contains_nan() const {
        return std::isnan(x) || std::isnan(y) || std::isnan(z);
    }

    T operator[](Axis3 ax) const {
        return ax == Axis3::X ? x : (ax == Axis3::Y ? y : z);
    }

    T& operator[](Axis3 ax) {
        return ax == Axis3::X ? x : (ax == Axis3::Y ? y : z);
    }

    T operator[](size_t i) const {
        assert(i < 3);
        return i == 0 ? x : (i == 1 ? y : z);
    }

    T& operator[](size_t i) {
        assert(i < 3);
        return i == 0 ? x : (i == 1 ? y : z);
    }

    bool operator==(const Vector3& v) const {
        return x == v.x && y == v.y && z == v.z;
    }

    bool operator!=(const Vector3& v) const { return !(*this == v); }

    Vector3<T> operator+(const Vector3<T>& v) const {
        return {x + v.x, y + v.y, z + v.z};
    }

    Vector3<T>& operator+=(const Vector3<T>& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Vector3<T> operator-(const Vector3<T>& v) const {
        return {x - v.x, y - v.y, z - v.z};
    }

    Vector3<T>& operator-=(const Vector3<T>& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Vector3<T> operator*(T s) const { return {s * x, s * y, s * z}; }

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

    Vector3<T>& operator/=(T s) {
        assert(s != 0);
        return *this *= 1 / s;
    }

    Vector3<T> operator-() const { return {-x, -y, -z}; }

    T length_squared() const { return x * x + y * y + z * z; }
    T length() const { return std::sqrt(length_squared()); }

    std::string to_string() const {
        return "[" + std::to_string(x) + ", " + std::to_string(y) + ", " +
               std::to_string(z) + "]";
    }

public:
    T x = 0;
    T y = 0;
    T z = 0;
};

using Vector3f = Vector3<float>;
using Vector3i = Vector3<int>;

template <typename T> Vector3<T> operator*(T s, const Vector3<T>& v) {
    return {s * v.x, s * v.y, s * v.z};
}

template <typename T> Vector3<T> abs(const Vector3<T>& v) {
    return {std::abs(v.x), std::abs(v.y), std::abs(v.z)};
}

template <typename T> T dot(const Vector3<T>& v1, const Vector3<T>& v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

template <typename T> T absdot(const Vector3<T>& v1, const Vector3<T>& v2) {
    return std::abs(dot(v1, v2));
}

template <typename T>
Vector3<T> cross(const Vector3<T>& v1, const Vector3<T>& v2) {
    // use double to prevent cancelation errors
    double v1x = v1.x;
    double v1y = v1.y;
    double v1z = v1.z;
    double v2x = v2.x;
    double v2y = v2.y;
    double v2z = v2.z;
    return {static_cast<float>((v1y * v2z) - (v1z * v2y)),
            static_cast<float>((v1z * v2x) - (v1x * v2z)),
            static_cast<float>((v1x * v2y) - (v1y * v2x))};
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
    return {std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z)};
}

template <typename T>
Vector3<T> max(const Vector3<T>& p1, const Vector3<T>& p2) {
    return {std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z)};
}

template <typename T>
Vector3<T> permute(const Vector3<T>& v, int x, int y, int z) {
    return {v[x], v[y], v[z]};
}

/**
 * Point in 2-dimensional space.
 */
template <typename T> class Point2 {
public:
    Point2() = default;
    Point2(T x, T y) : x{x}, y{y} { assert(!contains_nan()); }

    template <typename U>
    Point2(const Point2<U>& p)
        : x{static_cast<T>(p.x)}, y{static_cast<T>(p.y)} {
        assert(!contains_nan());
    }

    template <typename U>
    explicit Point2(const Vector2<U>& v)
        : x{static_cast<T>(v.x)}, y{static_cast<T>(v.y)} {
        assert(!contains_nan());
    }

    template <typename U> Point2<U> as() const { return Point2<U>(*this); }

    template <typename U> explicit operator Vector2<U>() const {
        return {x, y};
    }

    bool contains_nan() const { return std::isnan(x) || std::isnan(y); }

    T operator[](Axis2 ax) const { return ax == Axis2::X ? x : y; }

    T& operator[](Axis2 ax) { return ax == Axis2::X ? x : y; }

    T operator[](size_t i) const {
        assert(i < 2);
        return i == 0 ? x : y;
    }

    T& operator[](size_t i) {
        assert(i < 2);
        return i == 0 ? x : y;
    }

    bool operator==(const Point2<T>& p) const { return x == p.x && y == p.y; }

    bool operator!=(const Point2<T>& p) const { return !(*this == p); }

    Point2<T> operator+(const Vector2<T>& v) const {
        return {x + v.x, y + v.y};
    }

    Point2<T>& operator+=(const Vector2<T>& v) {
        x += v.x;
        y += v.y;
        return *this;
    }

    Point2<T> operator+(const Point2<T>& p) const { return {x + p.x, y + p.y}; }

    Point2<T>& operator+=(const Point2<T>& p) {
        x += p.x;
        y += p.y;
        return *this;
    }

    Vector2<T> operator-(const Point2<T>& p) const {
        return {x - p.x, y - p.y};
    }

    Point2<T> operator-(const Vector2<T>& v) const {
        return {x - v.x, y - v.y};
    }

    Point2<T>& operator-=(const Vector2<T>& v) {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    Point2<T> operator*(T s) const { return Point2<T>(s * x, s * y); }

    template <typename U> friend Point2<U> operator*(U s, const Point2<U>& p) {
        return {s * p.x, s * p.y};
    }

    Point2<T>& operator*=(T s) {
        x *= s;
        y *= s;
        return *this;
    }

    Point2<T> operator/(T s) const {
        assert(s != 0);
        return (1 / s) * *this;
    }

    Point2<T>& operator/=(T s) {
        assert(s != 0);
        return *this *= 1 / s;
    }

    std::string to_string() const {
        return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
    }

public:
    T x = 0;
    T y = 0;
};

using Point2f = Point2<float>;
using Point2i = Point2<int>;

template <typename T> float distance(const Point2<T>& p1, const Point2<T>& p2) {
    return (p1 - p2).length();
}

template <typename T>
float distance_squared(const Point2<T>& p1, const Point2<T>& p2) {
    return (p1 - p2).length_squared();
}

/**
 * Linearly interpolate between two points.
 */
template <typename T>
Point2<T> lerp(float t, const Point2<T>& p0, const Point2<T>& p1) {
    return (1 - t) * p0 + t * p1;
}

template <typename T> Point2<T> min(const Point2<T>& p1, const Point2<T>& p2) {
    return {std::min(p1.x, p2.x), std::min(p1.y, p2.y)};
}

template <typename T> Point2<T> max(const Point2<T>& p1, const Point2<T>& p2) {
    return {std::max(p1.x, p2.x), std::max(p1.y, p2.y)};
}

template <typename T> Point2<T> floor(const Point2<T>& p) {
    return {std::floor(p.x), std::floor(p.y)};
}

template <typename T> Point2<T> ceil(const Point2<T>& p) {
    return {std::ceil(p.x), std::ceil(p.y)};
}

template <typename T> Point2<T> abs(const Point2<T>& p) {
    return {std::abs(p.x), std::abs(p.y)};
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
        : x{static_cast<T>(p.x)}
        , y{static_cast<T>(p.y)}
        , z{static_cast<T>(p.z)} {
        assert(!contains_nan());
    }

    template <typename U> Point3<U> as() const { return Point3<U>(*this); }

    template <typename U>
    explicit Point3(const Vector3<U>& v)
        : x{static_cast<T>(v.x)}
        , y{static_cast<T>(v.y)}
        , z{static_cast<T>(v.z)} {
        assert(!contains_nan());
    }

    template <typename U> explicit operator Vector3<U>() const {
        return {x, y, z};
    }

    bool contains_nan() const {
        return std::isnan(x) || std::isnan(y) || std::isnan(z);
    }

    T operator[](Axis3 ax) const {
        return ax == Axis3::X ? x : (ax == Axis3::Y ? y : z);
    }

    T& operator[](Axis3 ax) {
        return ax == Axis3::X ? x : (ax == Axis3::Y ? y : z);
    }

    T operator[](size_t i) const {
        assert(i < 3);
        return i == 0 ? x : (i == 1 ? y : z);
    }

    T& operator[](size_t i) {
        assert(i < 3);
        return i == 0 ? x : (i == 1 ? y : z);
    }

    bool operator==(const Point3<T>& v) const {
        return x == v.x && y == v.y && z == v.z;
    }

    bool operator!=(const Point3<T>& v) const { return !(*this == v); }

    Point3<T> operator+(const Vector3<T>& v) const {
        return {x + v.x, y + v.y, z + v.z};
    }

    Point3<T>& operator+=(const Vector3<T>& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Point3<T> operator+(const Point3<T>& p) const {
        return {x + p.x, y + p.y, z + p.z};
    }

    Point3<T>& operator+=(const Point3<T>& p) {
        x += p.x;
        y += p.y;
        z += p.z;
        return *this;
    }

    Vector3<T> operator-(const Point3<T>& p) const {
        return {x - p.x, y - p.y, z - p.z};
    }

    Point3<T> operator-(const Vector3<T>& v) const {
        return {x - v.x, y - v.y, z - v.z};
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

    Point3<T>& operator/=(T s) {
        assert(s != 0);
        return *this *= 1 / s;
    }

    std::string to_string() const {
        return "(" + std::to_string(x) + ", " + std::to_string(y) + ", " +
               std::to_string(z) + ")";
    }

public:
    T x = 0;
    T y = 0;
    T z = 0;
};

using Point3f = Point3<float>;
using Point3i = Point3<int>;

template <typename T> Point3<T> operator*(T s, const Point3<T>& p) {
    return {s * p.x, s * p.y, s * p.z};
}

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
    return {std::min(p1.x, p2.x), std::min(p1.y, p2.y), std::min(p1.z, p2.z)};
}

template <typename T> Point3<T> max(const Point3<T>& p1, const Point3<T>& p2) {
    return {std::max(p1.x, p2.x), std::max(p1.y, p2.y), std::max(p1.z, p2.z)};
}

template <typename T> Point3<T> floor(const Point3<T>& p) {
    return {std::floor(p.x), std::floor(p.y), std::floor(p.z)};
}

template <typename T> Point3<T> ceil(const Point3<T>& p) {
    return {std::ceil(p.x), std::ceil(p.y), std::ceil(p.z)};
}

template <typename T> Point3<T> abs(const Point3<T>& p) {
    return {std::abs(p.x), std::abs(p.y), std::abs(p.z)};
}

template <typename T>
Point3<T> permute(const Point3<T>& p, int x, int y, int z) {
    return {p[x], p[y], p[z]};
}

/**
 * Normal in 3-dimensional space.
 */
template <typename T> class Normal3 {
public:
    Normal3() = default;
    Normal3(T x, T y, T z) : x{x}, y{y}, z{z} { assert(!contains_nan()); }

    explicit Normal3(const Vector3<T>& v) : x(v.x), y(v.y), z(v.z) {
        assert(!contains_nan());
    }

    template <typename U>
    explicit Normal3(const Vector3<U>& v)
        : x(static_cast<T>(v.x))
        , y(static_cast<T>(v.y))
        , z(static_cast<T>(v.z)) {
        assert(!contains_nan());
    }

    template <typename U>
    explicit Normal3(const Point3<U>& p)
        : x(static_cast<T>(p.x))
        , y(static_cast<T>(p.y))
        , z(static_cast<T>(p.z)) {
        assert(!contains_nan());
    }

    template <typename U> explicit operator Vector3<U>() const {
        return {x, y, z};
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

    bool operator==(const Normal3<T>& n) const {
        return x == n.x && y == n.y && z == n.z;
    }

    bool operator!=(const Normal3<T>& v) const { return !(*this == v); }

    Normal3<T> operator+(const Normal3<T>& v) const {
        return {x + v.x, y + v.y, z + v.z};
    }

    Normal3<T>& operator+=(const Normal3<T>& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Normal3<T> operator-(const Normal3<T>& v) const {
        return {x - v.x, y - v.y, z - v.z};
    }

    Normal3<T>& operator-=(const Normal3<T>& v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Normal3<T> operator*(T s) const { return Normal3<T>(s * x, s * y, s * z); }

    template <typename U>
    friend Normal3<U> operator*(U s, const Normal3<U>& n) {
        return {s * n.x, s * n.y, s * n.z};
    }

    Normal3<T>& operator*=(T s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    Normal3<T> operator/(T s) const {
        assert(s != 0);
        return (1 / s) * *this;
    }

    Normal3<T>& operator/=(T s) {
        assert(s != 0);
        return *this *= 1 / s;
    }

    T length_squared() const { return x * x + y * y + z * z; }
    T length() const { return std::sqrt(length_squared()); }

    std::string to_string() const {
        return "[" + std::to_string(x) + ", " + std::to_string(y) + ", " +
               std::to_string(z) + "]";
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

template <typename T> T dot(const Vector3<T>& v, const Normal3<T>& n) {
    return v.x * n.x + v.y * n.y + v.z * n.z;
}

template <typename T> T dot(const Normal3<T>& n, const Vector3<T>& v) {
    return n.x * v.x + n.y * v.y + n.z * v.z;
}

template <typename T> T absdot(const Normal3<T>& n1, const Normal3<T>& n2) {
    return std::abs(dot(n1, n2));
}

template <typename T> T absdot(const Vector3<T>& v, const Normal3<T>& n) {
    return std::abs(dot(v, n));
}

template <typename T> T absdot(const Normal3<T>& n, const Vector3<T>& v) {
    return std::abs(dot(n, v));
}

template <typename T> Normal3<T> normalize(const Normal3<T>& n) {
    return n / n.length();
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

/**
 * Bounding box in 2d space.
 */
template <typename T> class Bbox2 {
public:
    Bbox2()
        : p_min(std::numeric_limits<T>::lowest(),
                std::numeric_limits<T>::lowest())
        , p_max(std::numeric_limits<T>::max(), std::numeric_limits<T>::max()) {}
    Bbox2(const Point2<T>& p) : p_min(p), p_max(p) {}
    Bbox2(const Point2<T>& p1, const Point2<T>& p2)
        : p_min(min(p1, p2)), p_max(max(p1, p2)) {}

    template <typename U>
    explicit Bbox2(const Bbox2<U>& b) : p_min(b.p_min), p_max(b.p_max) {}

    template <typename U> Bbox2<U> as() const { return Bbox2<U>(*this); }

    const Point2<T>& operator[](size_t i) const {
        assert(i < 2);
        return i == 0 ? p_min : p_max;
    }

    Point2<T>& operator[](size_t i) {
        assert(i < 2);
        return i == 0 ? p_min : p_max;
    }

    Point2<T> corner(size_t corner) const {
        return {(*this)[(corner & 1)].x, (*this)[(corner & 2) ? 1 : 0].y};
    }

    bool empty() const { return p_max.x <= p_min.x && p_max.y <= p_min.y; }

    bool planar(Axis2 plane_ax) const {
        return std::abs(p_max[plane_ax] - p_min[plane_ax]) > EPS;
    }

    bool planar(size_t plane_ax) const {
        assert(plane_ax < 2);
        return std::abs(p_max[plane_ax] - p_min[plane_ax]) > EPS;
    }

    bool operator==(const Bbox2<T> other) const {
        return p_min == other.p_min && p_max == other.p_max;
    }
    bool operator!=(const Bbox2<T> other) const { return !(*this == other); }

    T width() const { return p_max.x - p_min.x; }
    T height() const { return p_max.y - p_min.y; }
    Vector2<T> diagonal() const { return p_max - p_min; }

    T area() const {
        Vector2<T> d = diagonal();
        return d.x * d.y;
    }

    Point2<T> lerp(const Point2f& t) const {
        return {static_cast<T>(turner::lerp(t.x, p_min.x, p_max.x)),
                static_cast<T>(turner::lerp(t.y, p_min.y, p_max.y))};
    }

    std::pair<Bbox2<T>, Bbox2<T>> split(size_t plane_ax,
                                        float plane_pos) const {
        assert(plane_ax < 2);
        assert(p_min[plane_ax] <= plane_pos);
        assert(plane_pos <= p_max[plane_ax]);

        Point2<T> lmax = p_max;
        lmax[plane_ax] = static_cast<T>(plane_pos);
        Point2<T> rmin = p_min;
        rmin[plane_ax] = plane_pos;

        return {{p_min, lmax}, {rmin, p_max}};
    }

    std::string to_string() const {
        return "[" + p_min.to_string() + ", " + p_max.to_string() + "]";
    }

public:
    Point2<T> p_min;
    Point2<T> p_max;
};

using Bbox2f = Bbox2<float>;
using Bbox2i = Bbox2<int>;

template <typename T>
Bbox2<T> bbox_union(const Bbox2<T>& b, const Point3<T>& p) {
    return {min(b.p_min, p), max(b.p_max, p)};
}

template <typename T>
Bbox2<T> bbox_union(const Bbox2<T>& b1, const Bbox2<T>& b2) {
    return {min(b1.p_min, b2.p_min), max(b1.p_max, b2.p_max)};
}

template <typename T>
Bbox2<T> intersect(const Bbox2<T>& b1, const Bbox2<T>& b2) {
    return {max(b1.p_min, b2.p_min), min(b1.p_max, b2.p_max)};
}

template <typename T> bool overlaps(const Bbox2<T>& b1, const Bbox2<T>& b2) {
    bool x = (b1.p_max.x >= b2.p_min.x) && (b1.p_min.x <= b2.p_max.x);
    bool y = (b1.p_max.y >= b2.p_min.y) && (b1.p_min.y <= b2.p_max.y);
    bool z = (b1.p_max.z >= b2.p_min.z) && (b1.p_min.z <= b2.p_max.z);
    return x && y && z;
}

template <typename T> bool inside(const Point2<T>& p, const Bbox2<T>& b) {
    return (p.x >= b.p_min.x && p.x <= b.p_max.x && p.y >= b.p_min.y &&
            p.y <= b.p_max.y && p.z >= b.p_min.z && p.z <= b.p_max.z);
}

/**
 * Iterator through points for integer-valued bounding box.
 */

class Bbox2iIterator : public std::forward_iterator_tag {
public:
    Bbox2iIterator(const Bbox2i& b, const Point2i& p) : bbox_(&b), p_(p) {}
    Bbox2iIterator operator++() {
        advance();
        return *this;
    }
    Bbox2iIterator operator++(int) {
        Bbox2iIterator previous = *this;
        advance();
        return previous;
    }
    bool operator==(const Bbox2iIterator& other) const {
        return bbox_ == other.bbox_ && p_ == other.p_;
    }
    bool operator!=(const Bbox2iIterator& other) const {
        return !(*this == other);
    }
    const Point2i& operator*() const { return p_; }

private:
    void advance() {
        ++p_.x;
        if (bbox_->p_max.x <= p_.x) {
            p_.x = bbox_->p_min.x;
            ++p_.y;
        }
    }

private:
    const Bbox2i* bbox_;
    Point2i p_;
};

inline Bbox2iIterator begin(const Bbox2i& b) {
    return Bbox2iIterator(b, b.p_min);
}

inline Bbox2iIterator end(const Bbox2i& b) {
    return b.empty() ? Bbox2iIterator(b, b.p_min)
                     : Bbox2iIterator(b, {b.p_min.x, b.p_max.y});
}

/**
 * Bounding box in 3d space.
 */
template <typename T> class Bbox3 {
public:
    Bbox3()
        : p_min(std::numeric_limits<T>::lowest(),
                std::numeric_limits<T>::lowest(),
                std::numeric_limits<T>::lowest())
        , p_max(std::numeric_limits<T>::max(), std::numeric_limits<T>::max(),
                std::numeric_limits<T>::max()) {}
    Bbox3(const Point3<T>& p) : p_min(p), p_max(p) {}
    Bbox3(const Point3<T>& p1, const Point3<T>& p2)
        : p_min(min(p1, p2)), p_max(max(p1, p2)) {}

    template <typename U>
    explicit Bbox3(const Bbox3<U>& b) : p_min(b.p_min), p_max(b.p_max) {}

    template <typename U> Bbox3<U> as() const { return Bbox3<U>(*this); }

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

    bool empty() const {
        return p_max.x <= p_min.x && p_max.y <= p_min.y && p_max.z <= p_min.z;
    }

    bool planar(Axis3 plane_ax) const {
        return std::abs(p_max[plane_ax] - p_min[plane_ax]) < EPS;
    }

    bool planar(size_t plane_ax) const {
        assert(plane_ax < 3);
        return std::abs(p_max[plane_ax] - p_min[plane_ax]) < EPS;
    }

    bool operator==(const Bbox3<T> other) const {
        return p_min == other.p_min && p_max == other.p_max;
    }

    bool operator!=(const Bbox3<T> other) const { return !(*this == other); }

    Vector3<T> diagonal() const { return p_max - p_min; }

    T surface_area() const {
        Vector3<T> d = diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    T volume() const {
        Vector3<T> d = diagonal();
        return d.x * d.y * d.z;
    }

    Point3<T> lerp(const Point3f& t) const {
        return {static_cast<T>(turner::lerp(t.x, p_min.x, p_max.x)),
                static_cast<T>(turner::lerp(t.y, p_min.y, p_max.y)),
                static_cast<T>(turner::lerp(t.z, p_min.z, p_max.z))};
    }

    std::pair<Bbox3<T>, Bbox3<T>> split(Axis3 plane_ax, float plane_pos) const {
        return split(static_cast<size_t>(plane_ax), plane_pos);
    }

    std::pair<Bbox3<T>, Bbox3<T>> split(size_t plane_ax,
                                        float plane_pos) const {
        assert(plane_ax < 3);
        assert(p_min[plane_ax] - EPS <= plane_pos);
        assert(plane_pos <= p_max[plane_ax] + EPS);

        Point3<T> lmax = p_max;
        lmax[plane_ax] = static_cast<T>(plane_pos);
        Point3<T> rmin = p_min;
        rmin[plane_ax] = plane_pos;

        return {{p_min, lmax}, {rmin, p_max}};
    }

    std::string to_string() const {
        return "[" + p_min.to_string() + ", " + p_max.to_string() + "]";
    }

public:
    Point3<T> p_min;
    Point3<T> p_max;
};

using Bbox3f = Bbox3<float>;
using Bbox3i = Bbox3<int>;

template <typename T>
Bbox3<T> bbox_union(const Bbox3<T>& b, const Point3<T>& p) {
    return {min(b.p_min, p), max(b.p_max, p)};
}

template <typename T>
Bbox3<T> bbox_union(const Bbox3<T>& b1, const Bbox3<T>& b2) {
    return {min(b1.p_min, b2.p_min), max(b1.p_max, b2.p_max)};
}

template <typename T>
Bbox3<T> intersect(const Bbox3<T>& b1, const Bbox3<T>& b2) {
    return {max(b1.p_min, b2.p_min), min(b1.p_max, b2.p_max)};
}

template <typename T> bool overlaps(const Bbox3<T>& b1, const Bbox3<T>& b2) {
    bool x = (b1.p_max.x >= b2.p_min.x) && (b1.p_min.x <= b2.p_max.x);
    bool y = (b1.p_max.y >= b2.p_min.y) && (b1.p_min.y <= b2.p_max.y);
    bool z = (b1.p_max.z >= b2.p_min.z) && (b1.p_min.z <= b2.p_max.z);
    return x && y && z;
}

template <typename T> bool inside(const Point3<T>& p, const Bbox3<T>& b) {
    return (p.x >= b.p_min.x && p.x <= b.p_max.x && p.y >= b.p_min.y &&
            p.y <= b.p_max.y && p.z >= b.p_min.z && p.z <= b.p_max.z);
}

//
// Explicit template initialization
//

template class Vector2<float>;
template class Vector2<int>;
template class Vector3<float>;
template class Vector3<int>;

template class Point2<float>;
template class Point2<int>;
template class Point3<float>;
template class Point3<int>;

template class Normal3<float>;

template class Bbox2<float>;
template class Bbox2<int>;
template class Bbox3<float>;
template class Bbox3<int>;

} // namespace turner
