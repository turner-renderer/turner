#include <assimp/types.h>

#include <vector>

#pragma once

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
        //TODO: Assert x < width and y < height
        return _image_data[y * height + x];
    }

    const aiColor4D& operator()(size_t x, size_t y) const {
        //TODO: Assert x < width and y < height
        return _image_data[y * height + x];
    }

    std::vector<aiColor4D>::const_iterator end() const {
        return _image_data.end();
    }
    std::vector<aiColor4D>::const_iterator begin() const {
        return _image_data.begin();
    }

private:

    std::vector<aiColor4D> _image_data;
};
