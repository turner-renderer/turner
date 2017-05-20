#pragma once

#include "geometry.h"
#include <string>

namespace turner {

template <typename Filter> class Film {
public:
    Film(const Point2i& resolution, const Bbox2f& crop_window, Filter filter,
         float diagonal_cm, const std::string& filename, float scale)
        : resolution(resolution)
        , diagonal_cm(diagonal_cm)
        , filter(filter)
        , filename(filename)
        , cropped_pixel_bbox(
              Point2i(std::ceil(resolution.x * crop_window.p_min.x),
                      std::ceil(resolution.y * crop_window.p_min.y)),
              Point2i(std::ceil(resolution.x * crop_window.p_max.x),
                      std::ceil(resolution.y * crop_window.p_max.y)))
        , scale_(scale) {}

public:
    const Point2i resolution;
    const float diagonal_cm;
    Filter filter;
    std::string filename;
    Bbox2i cropped_pixel_bbox;

private:
    float scale_;
};

} // namespace turner
