#pragma once

#include "matrix.h"
#include <cstddef>
#include <limits>

template <typename Iter, typename ValueFun>
auto min(Iter begin, Iter end, ValueFun&& get_value)
    -> decltype(get_value(*begin)) {
    using value_type = decltype(get_value(*begin));
    auto min = std::numeric_limits<value_type>::max();
    for (auto it = begin; it != end; ++it) {
        auto val = get_value(*it);
        if (val < min) {
            min = val;
        }
    }
    return min;
}

template <typename Iter, typename ValueFun>
auto max(Iter begin, Iter end, ValueFun&& get_value)
    -> decltype(get_value(*begin)) {
    using value_type = decltype(get_value(*begin));
    auto max = std::numeric_limits<value_type>::min();
    for (auto it = begin; it != end; ++it) {
        auto val = get_value(*it);
        if (max < val) {
            max = val;
        }
    }
    return max;
}

template <typename Iter, typename ValueFun>
auto average(Iter begin, Iter end, ValueFun&& get_value)
    -> decltype(get_value(*begin)) {
    using value_type = decltype(get_value(*begin));
    value_type avg{};
    size_t count = 0;
    for (auto it = begin; it != end; ++it) {
        avg += get_value(*it);
        count += 1;
    }
    return avg / count;
}

/**
 * Solves the equation Ax=b for vector x and returns result for x.
 *
 * Note that x should be initilized properly, e.g. with b.
 *
 * @param A matrix
 * @param x_0 initial guess for x
 * @param b vector
 */
template <typename Number>
math::Vector<Number>
gauss_seidel(const math::Matrix<Number>& A, const math::Vector<Number>& x_0,
             const math::Vector<Number>& b, const int max_iterations = 100) {
    auto x = x_0;
    const size_t rows = x.rows();
    for (int k = 0; k < max_iterations; ++k) {
        for (size_t i = 0; i < rows; ++i) {
            Number r = b(i);
            for (size_t j = 0; j < rows; ++j) {
                if (i != j) {
                    r -= x(j) * A(i, j);
                }
            }
            x(i) = r / A(i, i);
        }
    }
    return x;
}
