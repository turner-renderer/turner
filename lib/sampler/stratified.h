#pragma once

#include <vector>

// TODO: move out

struct Point2f {
    float x;
    float y;
};

struct Point2i {
    int x;
    int y;
};

class Sampler {
public:
    Sampler(size_t samples_per_pixel);

    void start_pixel(const Point2i& p);
    bool start_next_sample();
    bool set_sample_number(size_t sample_num);

public:
    size_t samples_per_pixel;

protected:
    Point2i current_pixel;
    size_t current_pixel_sample_index;
};

class PixelSampler : public Sampler {
public:
    PixelSampler(size_t samples_per_pixel, size_t num_sampled_dims);
};

class StratifiedSampler : public PixelSampler {
public:
    StratifiedSampler(size_t x_pixel_samples, size_t y_pixel_samples,
                      bool jitter_samples, size_t num_sampled_dims);

    void start_pixel(const Point2i& p);

public:
    size_t x_pixel_samples;
    size_t y_pixel_samples;
    bool jitter_samples;
};

//
// Implementation of Sampler
//

inline Sampler::Sampler(size_t samples_per_pixel)
    : samples_per_pixel(samples_per_pixel) {}

inline void Sampler::start_pixel(const Point2i& p) {
    current_pixel = p;
    current_pixel_sample_index = 0;
}

inline bool Sampler::start_next_sample() {
    return ++current_pixel_sample_index < samples_per_pixel;
}

inline bool Sampler::set_sample_number(size_t sample_num) {
    current_pixel_sample_index = sample_num;
    return current_pixel_sample_index < samples_per_pixel;
}

//
// Implementation of PixelSampler
//

inline PixelSampler::PixelSampler(size_t samples_per_pixel,
                                  size_t num_sampled_dims)
    : Sampler(samples_per_pixel) {}

//
// Implementation of StratifiedSampler
//

inline StratifiedSampler::StratifiedSampler(size_t x_pixel_samples,
                                            size_t y_pixel_samples,
                                            bool jitter_samples,
                                            size_t num_sampled_dims)
    : PixelSampler(x_pixel_samples * y_pixel_samples, num_sampled_dims)
    , x_pixel_samples(x_pixel_samples)
    , y_pixel_samples(y_pixel_samples)
    , jitter_samples(jitter_samples) {}

inline void StratifiedSampler::start_pixel(const Point2i& p) {
    //
    PixelSampler::start_pixel(p);
}
