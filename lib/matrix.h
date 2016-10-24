#pragma once

#include <cassert>
#include <initializer_list>
#include <vector>

namespace math {

template <typename Number> class Matrix {

public:
    Matrix(size_t rows, size_t cols)
        : rows_(rows), cols_(cols), data_(rows * cols) {}

    Matrix(size_t rows, size_t cols, std::initializer_list<Number> l)
        : rows_(rows), cols_(cols), data_(l) {
        assert(rows * cols == data_.size());
    }

    size_t cols() const { return cols_; }

    size_t rows() const { return rows_; }

    Number& operator()(size_t row, size_t col) {
        assert(0 <= row && row < rows_ && "Row index out of range.");
        assert(0 <= col && col < cols_ && "Column index out of range.");

        // We are row major
        return data_[row * cols_ + col];
    }

    const Number& operator()(size_t row, size_t col) const {
        assert(0 <= row && row < rows_ && "Row index out of range.");
        assert(0 <= col && col < cols_ && "Column index out of range.");

        // We are row major
        return data_[row * cols_ + col];
    }

private:
    size_t rows_;
    size_t cols_;
    std::vector<Number> data_;
};

template <typename Number> class Vector : public Matrix<Number> {
public:
    using Matrix<Number>::operator();

    Vector(size_t dim) : Matrix<Number>(dim, 1) {}
    Vector(size_t dim, std::initializer_list<Number> l)
        : Matrix<Number>(dim, 1, l) {}

    Number& operator()(size_t index) { return (*this)(index, 0); }

    const Number& operator()(size_t index) const { return (*this)(index, 0); }
};
} // namespace math
