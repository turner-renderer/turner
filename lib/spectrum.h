#include "turner.h"

#include <array>
#include <cassert>
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

    typename std::array<float, N>::iterator begin();
    typename std::array<float, N>::const_iterator begin() const;
    typename std::array<float, N>::iterator end();
    typename std::array<float, N>::const_iterator end() const;

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
Spectrum<N> lerp(float t, const Spectrum<N>& s1, const Spectrum<N>& s2);

template <size_t N>
std::ostream& operator<<(std::ostream& os, const Spectrum<N>& spectrum);

#include "spectrum.inl"

class SampledSpectrum : public Spectrum<60> {
public:
    static constexpr size_t LAMBDA_START = 400;
    static constexpr size_t LAMBDA_END = 700;

    SampledSpectrum(float v = 0);

    template <typename Iter>
    static SampledSpectrum from_samples(Iter sorted_samples_begin,
                                        Iter sorted_samples_end);
    static SampledSpectrum from_samples(
        std::initializer_list<std::pair<float /* lambda */, float /* v */>>
            sorted_samples);

    static SampledSpectrum X();
    static SampledSpectrum Y();
    static SampledSpectrum Z();

    std::tuple<float, float, float> to_xyz() const;
    float y() const;
};

//
// SampledSpectrum implementation
//

SampledSpectrum::SampledSpectrum(float v) : Spectrum<NUM_SAMPLES>(v) {}

namespace detail {

template <typename Iter>
float average_spectrum_samples(Iter sorted_samples_begin,
                               Iter sorted_samples_end, float lambda_start,
                               float lambda_end) {
    float sum = 0;

    float samples_lambda_start = sorted_samples_begin->first;
    float samples_lambda_end = (sorted_samples_end - 1)->first;
    float samples_value_start = sorted_samples_begin->second;
    float samples_value_end = (sorted_samples_end - 1)->second;

    // edge cases
    if (lambda_end <= samples_lambda_start) {
        return samples_value_start;
    } else if (samples_lambda_end <= lambda_start) {
        return samples_value_end;
    } else if (std::distance(sorted_samples_begin, sorted_samples_end) == 1) {
        return samples_value_start;
    }

    if (lambda_start < samples_lambda_start) {
        sum += samples_value_start * (samples_lambda_start - lambda_start);
    }
    if (samples_lambda_end < lambda_end) {
        sum += samples_value_end * (lambda_end - samples_lambda_end);
    }

    auto interpolate = [](float w, auto it, auto it_next) {
        float lambda_start = it->first;
        float lambda_end = it_next->first;
        return lerp((w - lambda_start) / (lambda_end - lambda_start),
                    it->second, it_next->second);
    };

    auto it = sorted_samples_begin;
    for (auto it_next = it + 1; it_next->first < lambda_start;
         it = it_next, ++it_next) {
        // advance to the first place in samples after lambda_start
        // Note: linear search is neglectable
    }
    for (auto it_next = it + 1;
         it_next != sorted_samples_end && it->first <= lambda_end;
         it = it_next, ++it_next) {
        float segment_lambda_start = std::max(lambda_start, it->first);
        float segment_lambda_end = std::min(lambda_end, it_next->first);
        sum += 0.5 * (interpolate(segment_lambda_start, it, it_next) +
                      interpolate(segment_lambda_end, it, it_next)) *
               (segment_lambda_end - segment_lambda_start);
    }

    return sum / (lambda_end - lambda_start);
}

} // namespace detail

template <typename Iter>
SampledSpectrum SampledSpectrum::from_samples(Iter sorted_samples_begin,
                                              Iter sorted_samples_end) {
    SampledSpectrum res;
    for (size_t i = 0; i < NUM_SAMPLES; ++i) {
        float lambda0 = lerp(1.f * i / NUM_SAMPLES, LAMBDA_START, LAMBDA_END);
        float lambda1 =
            lerp(1.f * (i + 1) / NUM_SAMPLES, LAMBDA_START, LAMBDA_END);
        res.c[i] = detail::average_spectrum_samples(
            sorted_samples_begin, sorted_samples_end, lambda0, lambda1);
    }
    return res;
}

SampledSpectrum SampledSpectrum::from_samples(
    std::initializer_list<std::pair<float /* lambda */, float /* v */>>
        sorted_samples) {
    return from_samples(sorted_samples.begin(), sorted_samples.end());
}

SampledSpectrum SampledSpectrum::X() {
    static auto X = SampledSpectrum::from_samples(CIE_X_SAMPLES.begin(),
                                                  CIE_X_SAMPLES.end());
    return X;
}
SampledSpectrum SampledSpectrum::Y() {
    static auto Y = SampledSpectrum::from_samples(CIE_Y_SAMPLES.begin(),
                                                  CIE_Y_SAMPLES.end());
    return Y;
}
SampledSpectrum SampledSpectrum::Z() {
    static auto Z = SampledSpectrum::from_samples(CIE_Z_SAMPLES.begin(),
                                                  CIE_Z_SAMPLES.end());
    return Z;
}

std::tuple<float, float, float> SampledSpectrum::to_xyz() const {
    float x = 0;
    float y = 0;
    float z = 0;

    const auto& X_spec = X();
    const auto& Y_spec = Y();
    const auto& Z_spec = Z();

    for (size_t i = 0; i < NUM_SAMPLES; ++i) {
        x += X_spec[i] * c[i];
        y += Y_spec[i] * c[i];
        z += Z_spec[i] * c[i];
    }
    float scale =
        1.f * (LAMBDA_END - LAMBDA_START) / (CIE_Y_INTEGRAL * NUM_SAMPLES);
    return {x * scale, y * scale, z * scale};
}

float SampledSpectrum::y() const {
    float yy = 0;
    const auto& Y_spec = Y();
    for (size_t i = 0; i < NUM_SAMPLES; ++i) {
        yy += Y_spec[i] * c[i];
    }
    return yy * (LAMBDA_END - LAMBDA_START) / (CIE_Y_INTEGRAL * NUM_SAMPLES);
}

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

template <size_t N>
typename std::array<float, N>::iterator Spectrum<N>::begin() {
    return c.begin();
}
template <size_t N>
typename std::array<float, N>::const_iterator Spectrum<N>::begin() const {
    return c.begin();
}
template <size_t N> typename std::array<float, N>::iterator Spectrum<N>::end() {
    return c.end();
}
template <size_t N>
typename std::array<float, N>::const_iterator Spectrum<N>::end() const {
    return c.end();
}

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
    assert(!has_nan());
    return *this;
}

template <size_t N>
Spectrum<N> Spectrum<N>::operator+(const Spectrum<N>& s) const {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = c[i] + s[i];
    }
    assert(!res.has_nan());
    return res;
}

template <size_t N> Spectrum<N>& Spectrum<N>::operator-=(const Spectrum<N>& s) {
    for (size_t i = 0; i < N; ++i) {
        c[i] -= s[i];
    }
    assert(!has_nan());
    return *this;
}

template <size_t N>
Spectrum<N> Spectrum<N>::operator-(const Spectrum<N>& s) const {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = c[i] - s[i];
    }
    assert(!res.has_nan());
    return res;
}

template <size_t N> Spectrum<N>& Spectrum<N>::operator*=(const Spectrum<N>& s) {
    for (size_t i = 0; i < N; ++i) {
        c[i] *= s[i];
    }
    assert(!has_nan());
    return *this;
}

template <size_t N>
Spectrum<N> Spectrum<N>::operator*(const Spectrum<N>& s) const {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = c[i] * s[i];
    }
    assert(!res.has_nan());
    return res;
}

template <size_t N> Spectrum<N>& Spectrum<N>::operator/=(const Spectrum<N>& s) {
    for (size_t i = 0; i < N; ++i) {
        c[i] /= s[i];
    }
    assert(!has_nan());
    return *this;
}

template <size_t N>
Spectrum<N> Spectrum<N>::operator/(const Spectrum<N>& s) const {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = c[i] / s[i];
    }
    assert(!res.has_nan());
    return res;
}

template <size_t N> Spectrum<N>& Spectrum<N>::operator*=(float a) {
    for (size_t i = 0; i < N; ++i) {
        c[i] *= a;
    }
    assert(!has_nan());
    return *this;
}

template <size_t N> Spectrum<N> Spectrum<N>::operator*(float a) const {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = c[i] * a;
    }
    assert(!res.has_nan());
    return res;
}

template <size_t N> Spectrum<N>& Spectrum<N>::operator/=(float a) {
    for (size_t i = 0; i < N; ++i) {
        c[i] /= a;
    }
    assert(!has_nan());
    return *this;
}

template <size_t N> Spectrum<N> Spectrum<N>::operator/(float a) const {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = c[i] / a;
    }
    assert(!res.has_nan());
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
    assert(!res.has_nan());
    return res;
}

template <size_t N> Spectrum<N> operator*(float a, const Spectrum<N>& s) {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = a * s[i];
    }
    assert(!res.has_nan());
    return res;
}

template <size_t N> Spectrum<N> pow(const Spectrum<N>& s, float e) {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = std::pow(s[i], e);
    }
    assert(!res.has_nan());
    return res;
}

template <size_t N> Spectrum<N> exp(const Spectrum<N>& s) {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = std::exp(s[i]);
    }
    assert(!res.has_nan());
    return res;
}

template <size_t N> Spectrum<N> sqrt(const Spectrum<N>& s) {
    Spectrum<N> res;
    for (size_t i = 0; i < N; ++i) {
        res[i] = std::sqrt(s[i]);
    }
    assert(!res.has_nan());
    return res;
}

template <size_t N>
Spectrum<N> lerp(float t, const Spectrum<N>& s1, const Spectrum<N>& s2) {
    return (1 - t) * s1 + t * s2;
}

template <size_t N>
std::ostream& operator<<(std::ostream& os, const Spectrum<N>& spectrum) {
    return os << spectrum.to_string();
}

} // namespace turner
