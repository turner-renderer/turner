#include "turner.h"

#include <array>
#include <cmath>
#include <ostream>

namespace turner {

/**
 * Base spectrum class.
 *
 * @tparam N Number of samples used to represent the spectrum.
 */
template <size_t N> class Spectrum {
public:
    static_assert(1 < N, "number of samples has to be greater than 0");
    static constexpr size_t NUM_SAMPLES = N;

    explicit Spectrum(float v = 0);

    // observers

    float& operator[](int i);
    float operator[](int i) const;

    bool operator==(const Spectrum& s) const;
    bool operator!=(const Spectrum& s) const;

    bool is_black() const;
    bool has_nan() const;

    // operations

    Spectrum& operator+=(const Spectrum& s);
    Spectrum operator+(const Spectrum& s) const;
    Spectrum& operator-=(const Spectrum& s);
    Spectrum operator-(const Spectrum& s) const;
    Spectrum& operator*=(const Spectrum& s);
    Spectrum operator*(const Spectrum& s) const;
    Spectrum& operator/=(const Spectrum& s);
    Spectrum operator/(const Spectrum& s) const;

    Spectrum& operator*=(float a);
    Spectrum operator*(float a) const;
    Spectrum& operator/=(float a);
    Spectrum operator/(float a) const;

    Spectrum clamp(float low = 0, float high = INFINITY) const;

    // utils

    std::string to_string() const;

protected:
    std::array<float, N> c;
};

template <size_t N> Spectrum<N> operator-(const Spectrum<N>& s);
template <size_t N> Spectrum<N> operator*(float a, const Spectrum<N>& s);
template <size_t N> Spectrum<N> pow(const Spectrum<N>& s, float e);
template <size_t N> Spectrum<N> exp(const Spectrum<N>& s);
template <size_t N> Spectrum<N> sqrt(const Spectrum<N>& s);

template <size_t N>
std::ostream& operator<<(std::ostream& os, const Spectrum<N>& spectrum);

//
// Spectrum implementation
//

template <size_t N> Spectrum<N>::Spectrum(float v) {
    for (size_t i = 0; i < N; ++i) {
        (*this)[i] = v;
    }
}

template <size_t N> float& Spectrum<N>::operator[](int i) { return c[i]; }
template <size_t N> float Spectrum<N>::operator[](int i) const { return c[i]; }

template <size_t N> bool Spectrum<N>::operator==(const Spectrum<N>& s) const {
    for (size_t i = 0; i < N; ++i) {
        if (c[i] != s[i]) {
            return false;
        }
    }
    return true;
}

template <size_t N> bool Spectrum<N>::operator!=(const Spectrum<N>& s) const {
    return !(*this == s);
}

template <size_t N> bool Spectrum<N>::is_black() const {
    for (size_t i = 0; i < N; ++i) {
        if (c[i] != 0.f) {
            return false;
        }
    }
    return true;
}

template <size_t N> bool Spectrum<N>::has_nan() const {
    for (size_t i = 0; i < N; ++i) {
        if (std::isnan(c[i])) {
            return true;
        }
    }
    return false;
}

template <size_t N> Spectrum<N>& Spectrum<N>::operator+=(const Spectrum<N>& s) {
    for (size_t i = 0; i < N; ++i) {
        c[i] += s[i];
    }
    return *this;
}

template <size_t N>
Spectrum<N> Spectrum<N>::operator+(const Spectrum<N>& s) const {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = c[i] + s[i];
    }
    return res;
}

template <size_t N> Spectrum<N>& Spectrum<N>::operator-=(const Spectrum<N>& s) {
    for (size_t i = 0; i < N; ++i) {
        c[i] -= s[i];
    }
    return *this;
}

template <size_t N>
Spectrum<N> Spectrum<N>::operator-(const Spectrum<N>& s) const {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = c[i] - s[i];
    }
    return res;
}

template <size_t N> Spectrum<N>& Spectrum<N>::operator*=(const Spectrum<N>& s) {
    for (size_t i = 0; i < N; ++i) {
        c[i] *= s[i];
    }
    return *this;
}

template <size_t N>
Spectrum<N> Spectrum<N>::operator*(const Spectrum<N>& s) const {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = c[i] * s[i];
    }
    return res;
}

template <size_t N> Spectrum<N>& Spectrum<N>::operator/=(const Spectrum<N>& s) {
    for (size_t i = 0; i < N; ++i) {
        c[i] /= s[i];
    }
    return *this;
}

template <size_t N>
Spectrum<N> Spectrum<N>::operator/(const Spectrum<N>& s) const {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = c[i] / s[i];
    }
    return res;
}

template <size_t N> Spectrum<N>& Spectrum<N>::operator*=(float a) {
    for (size_t i = 0; i < N; ++i) {
        c[i] *= a;
    }
    return *this;
}

template <size_t N> Spectrum<N> Spectrum<N>::operator*(float a) const {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = c[i] * a;
    }
    return res;
}

template <size_t N> Spectrum<N>& Spectrum<N>::operator/=(float a) {
    for (size_t i = 0; i < N; ++i) {
        c[i] /= a;
    }
    return *this;
}

template <size_t N> Spectrum<N> Spectrum<N>::operator/(float a) const {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = c[i] / a;
    }
    return res;
}

template <size_t N>
Spectrum<N> Spectrum<N>::clamp(float low, float high) const {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        if (c[i] < low) {
            res[i] = low;
        } else if (high < c[i]) {
            res[i] = high;
        } else {
            res[i] = c[i];
        }
    }
    return res;
}

template <size_t N> std::string Spectrum<N>::to_string() const {
    std::string str = "[";
    for (size_t i = 0; i < N - 1; ++i) {
        str += std::to_string(c[i]) + ", ";
    }
    str += std::to_string(c[N - 1]) + "]";
    return str;
}

template <size_t N> Spectrum<N> operator-(const Spectrum<N>& s) {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = -s[i];
    }
    return res;
}

template <size_t N> Spectrum<N> operator*(float a, const Spectrum<N>& s) {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = a * s[i];
    }
    return res;
}

template <size_t N> Spectrum<N> pow(const Spectrum<N>& s, float e) {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = std::pow(s[i], e);
    }
    return res;
}

template <size_t N> Spectrum<N> exp(const Spectrum<N>& s) {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = std::exp(s[i]);
    }
    return res;
}

template <size_t N> Spectrum<N> sqrt(const Spectrum<N>& s) {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = std::sqrt(s[i]);
    }
    return res;
}

template <size_t N>
std::ostream& operator<<(std::ostream& os, const Spectrum<N>& spectrum) {
    return os << spectrum.to_string();
}

} // namespace turner
