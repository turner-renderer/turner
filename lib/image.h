#pragma once

#include <assimp/types.h>

#include <assert.h>
#include <vector>

class Image {
public:
    Image(const size_t width, const size_t height)
        : width_(width), height_(height), image_data_(width * height) {}

    size_t width() const { return width_; }
    size_t height() const { return height_; }

    aiColor4D& operator()(size_t x, size_t y) {
        assert(x < width_ && "x out of image bounds");
        assert(y < height_ && "y out of image bounds");
        return image_data_[y * width_ + x];
    }

    const aiColor4D& operator()(size_t x, size_t y) const {
        assert(x < width_ && "x out of image bounds");
        assert(y < height_ && "y out of image bounds");
        return image_data_[y * width_ + x];
    }

    auto end() { return image_data_.end(); }
    auto begin() { return image_data_.begin(); }

    auto end() const { return image_data_.end(); }
    auto begin() const { return image_data_.begin(); }

private:
    size_t width_;
    size_t height_;
    std::vector<aiColor4D> image_data_;
};
