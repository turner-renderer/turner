#pragma once

#include "geometry.h"
#include "turner.h"

#include <iomanip>
#include <mutex>
#include <ostream>
#include <vector>

namespace turner {

/**
 * Cf. pixel filtering equation. contrib_sum is numerator,
 * and filter_weight_sum is denominator in the equation.
 */
template <typename Spectrum> struct FilmTilePixel {
    Spectrum contrib_sum = 0.f;
    float filter_weight_sum = 0.f;
};

template <typename Filter, typename Spectrum> class FilmTile {
public:
    FilmTile(const Bbox2i& pixel_bbox, Filter filter)
        : pixel_bbox_(pixel_bbox)
        , filter_(filter)
        , pixels_(std::max(0, pixel_bbox_.area())) {}

    FilmTilePixel<Spectrum>& get_pixel(const Point2i& p) {
        int width = pixel_bbox_.p_max.x - pixel_bbox_.p_min.x;
        int offset =
            (p.x - pixel_bbox_.p_min.x) + (p.y - pixel_bbox_.p_min.y) * width;
        assert(0 < offset && offset < pixel_bbox_.size());
        return pixels_[offset];
    }

    const Bbox2i& get_pixel_bbox() const { return pixel_bbox_; }

    /**
     * Add sample according to the pixel filtering equation:
     *
     *           ∑_i f(x - x_i, y - y_i) w(x_i, y_i) L(x_i, y_i)
     * I(x, y) = -----------------------------------------------,
     *                       ∑_i f(x - x_i, y - y_i)
     *
     * where:
     *     (x, y) - pixel coordinates,
     *     (x_i, y_i) - sample coordinates,
     *     I(x, y) - pixel value,
     *     f - filter,
     *     w(x_i, y_i) - sample weight,
     *     L(x_i, y_i) - sample radiance.
     *
     * @param p_film        sample in film == (x_i, y_i)
     * @param L             sample coordinates
     * @param sample_weight sample weight
     *
     * TODO: Possible improvement: Use a filter table with precomputed values.
     */
    void add_sample(const Point2f& p_film, const Spectrum& L,
                    float sample_weight = 1.f) {
        Point2f p_film_discrete = p_film - Vector2f(0.5f, 0.5f);
        Point2i p0 = ceil(p_film_discrete - filter_.radius);
        Point2i p1 = floor(p_film_discrete + filter_.radius) + Point2i(1, 1);
        p0 = max(p0, pixel_bbox_.p_min);
        p1 = min(p1, pixel_bbox_.p_max);

        for (int y = p0.y; y < p1.y; ++y) {
            for (int x = p0.x; x < p1.x; ++x) {
                float filter_weight = filter_(
                    Point2i(x - p_film_discrete.x, y - p_film_discrete.y));
                auto& pixel = get_pixel({x, y});
                pixel.contrib_sum += L * sample_weight * filter_weight;
                pixel.filter_weight_sum += filter_weight;
            }
        }
    }

private:
    Bbox2i pixel_bbox_;
    Filter filter_;
    std::vector<FilmTilePixel<Spectrum>> pixels_;
};

/**
 * Filter is a sample filter used to accumulate sampled values into a pixel.
 * Filter computes the weights of sampled values.
 */
template <typename Filter> class Film {
private:
    struct Pixel {
        float xyz[3] = {};
        float filter_weight_sum = 0;
    };

public:
    Film(const Point2i& resolution, const Bbox2f& crop_window, Filter filter,
         float scale)
        : resolution(resolution)
        , filter(filter)
        , cropped_pixel_bbox(
              Point2i(std::ceil(resolution.x * crop_window.p_min.x),
                      std::ceil(resolution.y * crop_window.p_min.y)),
              Point2i(std::ceil(resolution.x * crop_window.p_max.x),
                      std::ceil(resolution.y * crop_window.p_max.y)))
        , scale_(scale)
        , pixels_(cropped_pixel_bbox.area()) {}

    Bbox2i get_sample_bounds() const {
        Bbox2f float_bbox(floor(Point2f(cropped_pixel_bbox.p_min) +
                                Vector2f(0.5f, 0.5f) - filter.radius),
                          ceil(Point2f(cropped_pixel_bbox.p_max) -
                               Vector2f(0.5f, 0.5f) + filter.radius));
        return static_cast<Bbox2i>(float_bbox);
    }

    template <typename Spectrum>
    FilmTile<Filter, Spectrum> get_film_tile(const Bbox2i& sample_bbox) {
        Vector2f half_pixel(0.5f, 0.5f);
        auto float_bbox = static_cast<Bbox2f>(sample_bbox);
        Point2i p0 = ceil(float_bbox.p_min - half_pixel - filter.radius);
        Point2i p1 = floor(float_bbox.p_max - half_pixel + filter.radius) +
                     Point2i(1, 1);
        auto tile_pixel_bbox = intersect(Bbox2i(p0, p1), cropped_pixel_bbox);
        return FilmTile<Filter, Spectrum>(tile_pixel_bbox, filter);
    }

    template <typename Spectrum>
    void merge_film_tile(FilmTile<Filter, Spectrum> tile) {
        std::lock_guard<std::mutex> lock(merge_mutex_);
        for (const auto& p : tile.get_pixel_bbox()) {
            const auto& tile_pixel = tile.get_pixel(p);
            auto xyz = tile_pixel.contrib_sum.to_xyz();
            auto& pixel = get_pixel(p);
            pixel.xyz[0] += std::get<0>(xyz);
            pixel.xyz[1] += std::get<1>(xyz);
            pixel.xyz[2] += std::get<2>(xyz);
            pixel.filter_weight_sum += tile_pixel.filter_weight_sum;
        }
    }

    /**
     * Output sRGB image in PBM format.
     */
    friend std::ostream& operator<<(std::ostream& out,
                                    const Film<Filter>& film);

private:
    Pixel& get_pixel(const Point2i& p) {
        int width = cropped_pixel_bbox.width();
        int offset = (p.x - cropped_pixel_bbox.p_min.x) +
                     (p.y - cropped_pixel_bbox.p_min.y) * width;
        return pixels_[offset];
    }

public:
    const Point2i resolution;
    Filter filter;
    Bbox2i cropped_pixel_bbox;

private:
    float scale_;
    std::vector<Pixel> pixels_;
    std::mutex merge_mutex_;
};

template <typename Filter>
std::ostream& operator<<(std::ostream& out, const Film<Filter>& film) {
    std::vector<float> rgb(3 * film.cropped_pixel_bbox.area());
    int offset = 0;
    for (auto p : film.cropped_pixel_bbox) {
        auto& pixel = film.get_pixel(p);

        // xyz_to_rgb(pixel.xyz, &rgb[0]);  TODO: method missing

        // apply filter weight and normlize negative pixel value
        float filter_weight_sum = pixel.filter_weight_sum;
        if (filter_weight_sum != 0) {
            float inv_weight = 1.f / filter_weight_sum;
            rgb[3 * offset + 0] =
                std::max(0.f, rgb[3 * offset + 0] * inv_weight);
            rgb[3 * offset + 1] =
                std::max(0.f, rgb[3 * offset + 1] * inv_weight);
            rgb[3 * offset + 2] =
                std::max(0.f, rgb[3 * offset + 2] * inv_weight);
        }

        rgb[3 * offset + 0] *= film.scale_;
        rgb[3 * offset + 1] *= film.scale_;
        rgb[3 * offset + 2] *= film.scale_;

        ++offset;
    }

    output_pbm(out, std::move(rgb), film.cropped_pixel_bbox);
    return out;
}

void output_pbm(std::ostream& out, std::vector<float> rgb,
                const Bbox2i output_bbox) {
    Vector2i resolution = output_bbox.diagonal();

    // PBM header
    out << "P3" << std::endl;
    out << resolution.x << " " << resolution.y << std::endl;
    out << 255 << std::endl;

    auto to_byte = [](float v) -> int {
        return clamp(255.f * gamma_correction(v) + 0.5f, 0.f, 255.f);
    };

    for (int y = 0; y < resolution.y; ++y) {
        for (int x = 0; x < resolution.x; ++x) {
            out << std::setfill(' ') << std::setw(3)
                << to_byte(rgb[3 * (y * resolution.x + x) + 0]) << " "
                << std::setfill(' ') << std::setw(3)
                << to_byte(rgb[3 * (y * resolution.x + x) + 1]) << " "
                << std::setfill(' ') << std::setw(3)
                << to_byte(rgb[3 * (y * resolution.x + x) + 0]) << " ";
        }
        out << std::endl;
    }
}

} // namespace turner
