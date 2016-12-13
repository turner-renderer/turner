#pragma once
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
auto avg(Iter begin, Iter end, ValueFun&& get_value)
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

template <typename Iter, typename ValueFun>
auto avg(Iter begin, Iter end, ValueFun&& get_value,
         decltype(get_value(*begin)) init_val) -> decltype(get_value(*begin)) {
    auto avg = init_val;
    size_t count = 0;
    for (auto it = begin; it != end; ++it) {
        avg += get_value(*it);
        count += 1;
    }
    return avg / count;
}
