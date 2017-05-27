#include "../lib/spectrum.h"
#include <catch.hpp>

#include <random>
#include <sstream>

using namespace turner;

TEST_CASE("Constant black spectrum", "[spectrum]") {
    BaseSpectrum<64> s(0);
    REQUIRE(s.is_black());

    for (size_t i = 0; i < 64; ++i) {
        REQUIRE(s[i] == 0);
    }

    BaseSpectrum<64> t(0);
    BaseSpectrum<64> u(1);
    REQUIRE(!u.is_black());

    REQUIRE(s == t);
    REQUIRE(!(s != t));
    REQUIRE(!(s == u));
    REQUIRE(s != u);
}

TEST_CASE("NaN spectrum", "[spectrum]") {
    BaseSpectrum<64> s(0);
    s[0] /= 0;
    REQUIRE(s.has_nan());
}

TEST_CASE("Spectrum is a |R-algebra", "[spectrum]") {
    BaseSpectrum<2> o(0);
    BaseSpectrum<2> e(1);
    BaseSpectrum<2> e_neg(-1);

    BaseSpectrum<2> s(0);
    s[0] = -1;
    s[1] = 1;

    BaseSpectrum<2> t(0);
    t[0] = 1;
    t[1] = -1;

    BaseSpectrum<2> s2(0);
    s2[0] = -2;
    s2[1] = 2;

    REQUIRE(s + t == o);
    REQUIRE(t + s == o);
    REQUIRE(s + o == s);
    REQUIRE(o + s == s);
    REQUIRE(s - t == s2);
    REQUIRE(t - s == -s2);

    REQUIRE(2 * s == s2);
    REQUIRE(s * 2 == s2);

    REQUIRE(s2 / 2 == s);

    REQUIRE(s * t == e_neg);
    REQUIRE(s * t == -e);
    REQUIRE(s / t == e_neg);
    REQUIRE(t / s == -e);

    // inplace

    s[0] = -1;
    s[1] = 1;
    s += t;
    REQUIRE(s == o);

    s[0] = -1;
    s[1] = 1;
    s -= t;
    REQUIRE(s == s2);

    s[0] = -1;
    s[1] = 1;
    s *= t;
    REQUIRE(s == e_neg);

    s[0] = -1;
    s[1] = 1;
    s /= t;
    REQUIRE(s == e_neg);

    s[0] = -1;
    s[1] = 1;
    s *= 2;
    REQUIRE(s == s2);

    s[0] = -1;
    s[1] = 1;
    s2 /= 2;
    REQUIRE(s == s2);
}

TEST_CASE("Spectrum::to_string", "[spectrum]") {
    BaseSpectrum<4> s(1);
    REQUIRE(s.to_string() == "[1.000000, 1.000000, 1.000000, 1.000000]");

    std::stringstream ss;
    ss << s;
    REQUIRE(ss.str() == "[1.000000, 1.000000, 1.000000, 1.000000]");
}

TEST_CASE("Spectrum lerp", "[spectrum]") {
    BaseSpectrum<2> o(0);

    BaseSpectrum<2> s(0);
    s[0] = -1;
    s[1] = 1;

    BaseSpectrum<2> t(0);
    t[0] = 1;
    t[1] = -1;

    REQUIRE(lerp(0, s, t) == s);
    REQUIRE(lerp(1.f / 2, s, t) == o);
    REQUIRE(lerp(1, s, t) == t);
}

TEST_CASE("Spectrum::clamp", "[spectrum]") {
    BaseSpectrum<2> s(0);
    s[0] = -1;
    s[1] = 1;
    BaseSpectrum<2> t = s;

    t = s.clamp(0);
    REQUIRE(t[0] == 0);
    REQUIRE(t[1] == 1);

    t = s.clamp(-1, 0);
    REQUIRE(t[0] == -1);
    REQUIRE(t[1] == 0);

    t = s.clamp(0, 0);
    REQUIRE(t[0] == 0);
    REQUIRE(t[1] == 0);
}

TEST_CASE("Spectrum pow", "[spectrum]") {
    BaseSpectrum<2> s(0);
    s[0] = -1;
    s[1] = 1;
    auto t = pow(s, 2);

    REQUIRE(t[0] == std::pow(-1, 2));
    REQUIRE(t[1] == std::pow(1, 2));
}

TEST_CASE("Spectrum exp", "[spectrum]") {
    BaseSpectrum<2> s(0);
    s[0] = std::log(2.f);
    s[1] = std::log(3.f);
    auto t = exp(s);

    REQUIRE(t[0] == 2);
    REQUIRE(t[1] == 3);
}

TEST_CASE("Spectrum sqrt", "[spectrum]") {
    BaseSpectrum<2> s(0);
    s[0] = 0;
    s[1] = 1;
    auto t = sqrt(s);

    REQUIRE(t[0] == 0);
    REQUIRE(t[1] == 1);
}

TEST_CASE("SampledSpectrum::from_samples trivially constant", "[spectrum]") {
    auto spec = SampledSpectrum::from_samples({{400, 1}});
    for (const auto& c : spec) {
        REQUIRE(c == 1);
    }

    spec = SampledSpectrum::from_samples({{400, 1}});
    for (const auto& c : spec) {
        REQUIRE(c == 1);
    }

    spec = SampledSpectrum::from_samples({{300, 1}});
    for (const auto& c : spec) {
        REQUIRE(c == 1);
    }

    spec = SampledSpectrum::from_samples({{800, 1}});
    for (const auto& c : spec) {
        REQUIRE(c == 1);
    }
}

TEST_CASE("SampledSpectrum::from_samples constant", "[spectrum]") {
    auto spec = SampledSpectrum::from_samples({{400, 1}, {700, 1}});
    for (const auto& c : spec) {
        REQUIRE(c == 1);
    }

    spec = SampledSpectrum::from_samples({{300, 1}, {700, 1}});
    for (const auto& c : spec) {
        REQUIRE(c == 1);
    }

    spec = SampledSpectrum::from_samples({{400, 1}, {800, 1}});
    for (const auto& c : spec) {
        REQUIRE(c == 1);
    }
}

TEST_CASE("SampledSpectrum::from_samples identity sampling", "[spectrum]") {
    static std::default_random_engine gen(42);
    static std::uniform_real_distribution<float> rnd(0, 2);
    std::array<std::pair<float, float>, SampledSpectrum::NUM_SAMPLES> samples;
    const float step =
        1.f * (SampledSpectrum::LAMBDA_END - SampledSpectrum::LAMBDA_START) /
        SampledSpectrum::NUM_SAMPLES;
    for (size_t i = 0; i < SampledSpectrum::NUM_SAMPLES; ++i) {
        samples[i] = {SampledSpectrum::LAMBDA_START + i * step, rnd(gen)};
    }

    auto spec = SampledSpectrum::from_samples(samples.begin(), samples.end());
    for (size_t i = 0; i < SampledSpectrum::NUM_SAMPLES - 1; ++i) {
        float expected = (samples[i + 1].second + samples[i].second) / 2.f;
        REQUIRE(spec[i] == Approx(expected));
    }
}
