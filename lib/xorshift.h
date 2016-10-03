#pragma once

#include <assert.h>
#include <iomanip>
#include <stdint.h>
#include <type_traits>

inline constexpr uint64_t bitmask(int bits) {
    if (bits == 0) {
        return 0;
    }
    return (bitmask(bits - 1) << 1) + 1;
}


template<typename T>
class xorshift64star;

template<>
class xorshift64star<uint64_t> {
public:
    constexpr explicit xorshift64star(uint64_t seed) : seed_(seed) {
        assert(seed != 0);
    }
    uint64_t operator()() {
        seed_ ^= seed_ >> 12; // a
        seed_ ^= seed_ << 25; // b
        seed_ ^= seed_ >> 27; // c
        return seed_ * 2685821657736338717ULL;
    }

private:
    uint64_t seed_;
};


template<typename T>
class xorshift64star {
public:
    constexpr explicit xorshift64star(uint64_t seed) : gen_(seed) {
        assert(seed != 0);
    }

    T operator()() {
        return std::ldexp(static_cast<T>(gen_() & MANTISSA_MASK), -DIGITS);
    }

    static constexpr int DIGITS = std::numeric_limits<T>::digits;
    static constexpr uint64_t MANTISSA_MASK = bitmask(DIGITS);

    static_assert(std::is_floating_point<T>::value,
        "T expected to be a floating point type");
    static_assert(DIGITS <= 64, "T is too big");

private:
    xorshift64star<uint64_t> gen_;
};
