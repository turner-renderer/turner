#include "functional.h"
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

enum class SpectrumType { Reflectance, Illuminance };

class SampledSpectrum : public Spectrum<60> {
public:
    static constexpr size_t LAMBDA_START = 400;
    static constexpr size_t LAMBDA_END = 700;

    explicit SampledSpectrum(float v = 0);
    explicit SampledSpectrum(const Spectrum<60>& s);

    template <typename Iter>
    static SampledSpectrum from_samples(Iter sorted_samples_begin,
                                        Iter sorted_samples_end);
    static SampledSpectrum from_samples(
        std::initializer_list<std::pair<float /* lambda */, float /* v */>>
            sorted_samples);

    static const SampledSpectrum& X();
    static const SampledSpectrum& Y();
    static const SampledSpectrum& Z();

    static const SampledSpectrum& refl_d65_white();
    static const SampledSpectrum& refl_cyan();
    static const SampledSpectrum& refl_magenta();
    static const SampledSpectrum& refl_red();
    static const SampledSpectrum& refl_green();
    static const SampledSpectrum& refl_blue();
    static const SampledSpectrum& refl_yellow();

    static const SampledSpectrum& illum_d65_white();
    static const SampledSpectrum& illum_cyan();
    static const SampledSpectrum& illum_magenta();
    static const SampledSpectrum& illum_red();
    static const SampledSpectrum& illum_green();
    static const SampledSpectrum& illum_blue();
    static const SampledSpectrum& illum_yellow();

    float y() const;
    std::array<float, 3> to_xyz() const;
    std::array<float, 3> to_rgb() const;

    static SampledSpectrum from_rgb(float r, float g, float b,
                                    SpectrumType type);
    static SampledSpectrum
    from_xyz(float r, float g, float b,
             SpectrumType type = SpectrumType::Reflectance);
};

inline std::array<float, 3> xyz_to_rgb(float x, float y, float z);
inline std::array<float, 3> rgb_to_xyz(float r, float g, float b);

class RGBSpectrum : public Spectrum<3> {
public:
    explicit RGBSpectrum(float v = 0.f);
    explicit RGBSpectrum(const Spectrum<3>& s);

    // TODO: from_samples is not implemented

    float y() const;
    std::array<float, 3> to_xyz() const;
    std::array<float, 3> to_rgb() const;

    static RGBSpectrum from_rgb(float r, float g, float b,
                                SpectrumType type = SpectrumType::Reflectance);
    static RGBSpectrum from_xyz(float r, float g, float b,
                                SpectrumType type = SpectrumType::Reflectance);
};

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

//
// SampledSpectrum implementation
//

SampledSpectrum::SampledSpectrum(float v) : Spectrum<NUM_SAMPLES>(v) {}
SampledSpectrum::SampledSpectrum(const Spectrum<60>& s) : Spectrum<60>(s) {}

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

namespace detail {
template <size_t N>
SampledSpectrum zip_spectrum(const std::array<float, N>& frequencies,
                             const std::array<float, N>& values) {
    auto begin = make_zip_iterator(std::begin(frequencies), std::begin(values));
    auto end = make_zip_iterator(std::end(frequencies), std::end(values));
    return SampledSpectrum::from_samples(begin, end);
}
} // namespace detail

const SampledSpectrum& SampledSpectrum::X() {
    static auto s = detail::zip_spectrum(CIE_FREQUENCIES, CIE_X_SAMPLES);
    return s;
}

const SampledSpectrum& SampledSpectrum::Y() {
    static auto s = detail::zip_spectrum(CIE_FREQUENCIES, CIE_Y_SAMPLES);
    return s;
}

const SampledSpectrum& SampledSpectrum::Z() {
    static auto s = detail::zip_spectrum(CIE_FREQUENCIES, CIE_Z_SAMPLES);
    return s;
}

const SampledSpectrum& SampledSpectrum::refl_d65_white() {
    static auto s = detail::zip_spectrum(RGB_FREQUENCIES, RGB_REFL_WHITE);
    return s;
}

const SampledSpectrum& SampledSpectrum::refl_cyan() {
    static auto s = detail::zip_spectrum(RGB_FREQUENCIES, RGB_REFL_CYAN);
    return s;
}

const SampledSpectrum& SampledSpectrum::refl_magenta() {
    static auto s = detail::zip_spectrum(RGB_FREQUENCIES, RGB_REFL_MAGENTA);
    return s;
}

const SampledSpectrum& SampledSpectrum::refl_red() {
    static auto s = detail::zip_spectrum(RGB_FREQUENCIES, RGB_REFL_RED);
    return s;
}

const SampledSpectrum& SampledSpectrum::refl_green() {
    static auto s = detail::zip_spectrum(RGB_FREQUENCIES, RGB_REFL_GREEN);
    return s;
}

const SampledSpectrum& SampledSpectrum::refl_blue() {
    static auto s = detail::zip_spectrum(RGB_FREQUENCIES, RGB_REFL_BLUE);
    return s;
}

const SampledSpectrum& SampledSpectrum::refl_yellow() {
    static auto s = detail::zip_spectrum(RGB_FREQUENCIES, RGB_REFL_YELLOW);
    return s;
}

const SampledSpectrum& SampledSpectrum::illum_d65_white() {
    static auto s = detail::zip_spectrum(RGB_FREQUENCIES, RGB_ILLUM_WHITE);
    return s;
}

const SampledSpectrum& SampledSpectrum::illum_cyan() {
    static auto s = detail::zip_spectrum(RGB_FREQUENCIES, RGB_ILLUM_CYAN);
    return s;
}

const SampledSpectrum& SampledSpectrum::illum_magenta() {
    static auto s = detail::zip_spectrum(RGB_FREQUENCIES, RGB_ILLUM_MAGENTA);
    return s;
}

const SampledSpectrum& SampledSpectrum::illum_red() {
    static auto s = detail::zip_spectrum(RGB_FREQUENCIES, RGB_ILLUM_RED);
    return s;
}

const SampledSpectrum& SampledSpectrum::illum_green() {
    static auto s = detail::zip_spectrum(RGB_FREQUENCIES, RGB_ILLUM_GREEN);
    return s;
}

const SampledSpectrum& SampledSpectrum::illum_blue() {
    static auto s = detail::zip_spectrum(RGB_FREQUENCIES, RGB_ILLUM_BLUE);
    return s;
}

const SampledSpectrum& SampledSpectrum::illum_yellow() {
    static auto s = detail::zip_spectrum(RGB_FREQUENCIES, RGB_ILLUM_YELLOW);
    return s;
    return s;
}

std::array<float, 3> SampledSpectrum::to_xyz() const {
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
    x *= scale;
    y *= scale;
    z *= scale;

    return {x, y, z};
}

float SampledSpectrum::y() const {
    float yy = 0;
    const auto& Y_spec = Y();
    for (size_t i = 0; i < NUM_SAMPLES; ++i) {
        yy += Y_spec[i] * c[i];
    }
    return yy * (LAMBDA_END - LAMBDA_START) / (CIE_Y_INTEGRAL * NUM_SAMPLES);
}

std::array<float, 3> SampledSpectrum::to_rgb() const {
    auto xyz = to_xyz();
    return xyz_to_rgb(xyz[0], xyz[1], xyz[2]);
}

SampledSpectrum SampledSpectrum::from_rgb(float r, float g, float b,
                                          SpectrumType type) {
    SampledSpectrum s;
    if (type == SpectrumType::Reflectance) {
        if (r <= g && r <= b) {
            s += r * refl_d65_white();
            if (g <= b) {
                s += (g - r) * refl_cyan();
                s += (b - g) * refl_blue();
            } else {
                s += (b - r) * refl_cyan();
                s += (g - r) * refl_green();
            }
        } else if (g <= r && g <= b) {
            s += g * refl_d65_white();
            if (r <= b) {
                s += (r - g) * refl_magenta();
                s += (b - r) * refl_blue();
            } else {
                s += (b - g) * refl_magenta();
                s += (r - b) * refl_red();
            }
        } else {
            s += b * refl_d65_white();
            if (r <= g) {
                s += (r - b) * refl_yellow();
                s += (g - r) * refl_green();
            } else {
                s += (g - b) * refl_yellow();
                s += (r - g) * refl_red();
            }
        }
    } else if (type == SpectrumType::Illuminance) {
        if (r <= g && r <= b) {
            s += r * illum_d65_white();
            if (g <= b) {
                s += (g - r) * illum_cyan();
                s += (b - g) * illum_blue();
            } else {
                s += (b - r) * illum_cyan();
                s += (g - r) * illum_green();
            }
        } else if (g <= r && g <= b) {
            s += g * illum_d65_white();
            if (r <= b) {
                s += (r - g) * illum_magenta();
                s += (b - r) * illum_blue();
            } else {
                s += (b - g) * illum_magenta();
                s += (r - b) * illum_red();
            }
        } else {
            s += b * illum_d65_white();
            if (r <= g) {
                s += (r - b) * illum_yellow();
                s += (g - r) * illum_green();
            } else {
                s += (g - b) * illum_yellow();
                s += (r - g) * illum_red();
            }
        }
    } else {
        assert(!"logic error");
    }

    return s;
}

SampledSpectrum SampledSpectrum::from_xyz(float x, float y, float z,
                                          SpectrumType type) {
    auto rgb = xyz_to_rgb(x, y, z);
    return from_rgb(rgb[0], rgb[1], rgb[2], type);
}

/**
 * Matrix coefficients are from pbrt v3.
 */
inline std::array<float, 3> xyz_to_rgb(float x, float y, float z) {
    auto r = 3.240479f * x - 1.537150f * y - 0.498535f * z;
    auto g = -0.969256f * x + 1.875991f * y + 0.041556f * z;
    auto b = 0.055648f * x - 0.204043f * y + 1.057311f * z;
    return {r, g, b};
}

inline std::array<float, 3> rgb_to_xyz(float r, float g, float b) {
    auto x = 0.412453f * r + 0.357580f * g + 0.180423f * b;
    auto y = 0.212671f * r + 0.715160f * g + 0.072169f * b;
    auto z = 0.019334f * r + 0.119193f * g + 0.950227f * b;
    return {x, y, z};
}

//
// RGBSpectrum implementation
//

RGBSpectrum::RGBSpectrum(float v) : Spectrum<3>(v){};
RGBSpectrum::RGBSpectrum(const Spectrum<3>& s) : Spectrum<3>(s){};

float RGBSpectrum::y() const {
    // cf. rgb_to_xyz function for the coefficients
    return 0.212671f * c[0] + 0.715160f * c[1] + 0.072169f * c[2];
}

std::array<float, 3> RGBSpectrum::to_xyz() const {
    return rgb_to_xyz(c[0], c[1], c[2]);
}

std::array<float, 3> RGBSpectrum::to_rgb() const { return {c[0], c[1], c[2]}; }

RGBSpectrum RGBSpectrum::from_rgb(float r, float g, float b, SpectrumType) {
    RGBSpectrum s;
    s[0] = r;
    s[1] = g;
    s[2] = b;
    return s;
}

RGBSpectrum RGBSpectrum::from_xyz(float x, float y, float z, SpectrumType) {
    auto rgb = xyz_to_rgb(x, y, z);
    RGBSpectrum s;
    s[0] = rgb[0];
    s[1] = rgb[1];
    s[2] = rgb[2];
    return s;
}

} // namespace turner
