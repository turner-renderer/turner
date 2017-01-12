#pragma once
#include "types.h"

#include <assert.h>
#include <cmath>
#include <iomanip>
#include <ostream>
#include <vector>

/**
 * Bresenham's line algorithm
 * @param x0, y0    line start
 * @param x1, y1    line end
 * @param set_pixel callback function: (int, int) -> void, which sets pixel
 *                  at the given position
 */
template <typename SetPixelFun>
void bresenham(int x0, int y0, int x1, int y1, SetPixelFun&& set_pixel) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2; /* error value e_xy */

    while (1) {
        set_pixel(x0, y0);
        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2 * err;
        if (e2 > dy) { // e_xy+e_x > 0
            err += dy;
            x0 += sx;
        }
        if (e2 < dx) { // e_xy+e_y < 0
            err += dx;
            y0 += sy;
        }
    }
}

/**
 * Represents a raster image.
 */
class Image {
public:
    Image(const size_t width, const size_t height)
        : width_(width), height_(height), image_data_(width * height) {}

    size_t width() const { return width_; }
    size_t height() const { return height_; }

    Color& operator()(size_t x, size_t y) {
        assert(x < width_ && "x out of image bounds");
        assert(y < height_ && "y out of image bounds");
        return image_data_[y * width_ + x];
    }

    const Color& operator()(size_t x, size_t y) const {
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
    std::vector<Color> image_data_;
};

/**
 * Output image in PBM format.
 *
 * Cf. https://en.wikipedia.org/wiki/Netpbm_format#PPM_example.
 */
inline std::ostream& operator<<(std::ostream& os, const Image& img) {
    os << "P3" << std::endl;
    os << img.width() << " " << img.height() << std::endl;
    os << 255;
    int i = 0;
    for (auto color : img) {
        if (i++ % img.width() == 0) {
            os << std::endl;
        } else {
            os << " ";
        }

        os << std::setfill(' ') << std::setw(3)
           << static_cast<int>(clamp(255 * color.r * color.a)) << " "
           << std::setfill(' ') << std::setw(3)
           << static_cast<int>(clamp(255 * color.g * color.a)) << " "
           << std::setfill(' ') << std::setw(3)
           << static_cast<int>(clamp(255 * color.b * color.a));
    }

    return os;
}
