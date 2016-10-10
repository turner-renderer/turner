#pragma once

#include <assimp/types.h>

#include <assert.h>
#include <vector>

class Image {
public:
    const size_t width;
    const size_t height;

    Image(const size_t width, const size_t height)
        : width(width)
        , height(height)
        , _image_data(width * height)
    {}

    aiColor4D& operator()(size_t x, size_t y) {
        assert(x < width && "x out of image bounds");
        assert(y < height && "y out of image bounds");
        return _image_data[y * width + x];
    }

    const aiColor4D& operator()(size_t x, size_t y) const {
        assert(x < width && "x out of image bounds");
        assert(y < height && "y out of image bounds");
        return _image_data[y * width + x];
    }

    auto end() const {
        return _image_data.end();
    }
    auto begin() const {
        return _image_data.begin();
    }

    auto end() {
        return _image_data.end();
    }
    auto begin() {
        return _image_data.begin();
    }

private:

    std::vector<aiColor4D> _image_data;
};
