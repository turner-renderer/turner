#pragma once

#include <cstddef>

template <typename T> class Range {
public:
    Range(T* collection, std::ptrdiff_t size)
        : collection_(collection), size_(size) {}

    T* begin() { return &collection_[0]; }
    T* end() { return &collection_[size_]; }

private:
    T* collection_;
    size_t size_;
};

template <typename T> Range<T> make_range(T* collection, std::ptrdiff_t size) {
    return Range<T>(collection, size);
}
