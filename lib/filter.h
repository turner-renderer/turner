#pragma once

#include "geometry.h"
#include "turner.h"

namespace turner {

class BaseFilter {
public:
    BaseFilter(const Vector2f& radius)
        : radius(radius), inv_radius({1 / radius.x, 1 / radius.y}) {}

protected:
    Vector2f radius;
    Vector2f inv_radius;
};

class BoxFilter : public BaseFilter {
public:
    BoxFilter(const Vector2f& radius) : BaseFilter(radius) {}
    float operator()(const Point2f&) const { return 1.f; }
};

class TriangleFilter : public BaseFilter {
public:
    TriangleFilter(const Vector2f& radius) : BaseFilter(radius) {}
    float operator()(const Point2f& p) const {
        return std::max(0.f, radius.x - std::abs(p.x)) *
               std::max(0.f, radius.y - std::abs(p.y));
    }
};

class GaussianFilter : public BaseFilter {
public:
    GaussianFilter(const Vector2f& radius, float alpha)
        : BaseFilter(radius)
        , alpha_(alpha)
        , exp_x_(std::exp(-alpha_ * radius.x * radius.x))
        , exp_y_(std::exp(-alpha_ * radius.y * radius.y)) {}

    float operator()(const Point2f& p) const {
        return gaussian(p.x, exp_x_) * gaussian(p.y, exp_y_);
    }

    float gaussian(float x, float expv) const {
        return std::max<float>(0, std::exp(-alpha_ * x * x) - expv);
    }

private:
    float alpha_;
    float exp_x_;
    float exp_y_;
};

class MitchellFilter : public BaseFilter {
public:
    // Recommended values for b, c to have them along the line b + 2c = 1.
    MitchellFilter(const Vector2f& radius, float b, float c)
        : BaseFilter(radius), b_(b), c_(c) {}

    float operator()(const Point2f& p) const {
        return mitchell_1d(p.x * inv_radius.x) *
               mitchell_1d(p.y * inv_radius.y);
    }

    float mitchell_1d(float x) const {
        x = std::abs(2 * x);
        if (x > 1) {
            return ((-b_ - 6 * c_) * x * x * x + (6 * b_ + 30 * c_) * x * x +
                    (-12 * b_ - 49 * c_) * x + (8 * b_ + 24 * c_)) /
                   6.f;
        } else {
            return ((12 - 9 * b_ - 6 * c_) * x * x * x +
                    (-18 + 12 * b_ + 6 * c_) * x * x + (6 - 2 * b_)) /
                   6.f;
        }
    }

private:
    float b_;
    float c_;
};

class LanszosSincFilter : public BaseFilter {
public:
    // Recommended values for b, c are on the line b + 2c = 1.
    LanszosSincFilter(const Vector2f& radius, float tau)
        : BaseFilter(radius), tau_(tau) {}

    float operator()(const Point2f& p) const {
        return windowed_sinc(p.x, radius.x) * windowed_sinc(p.y, radius.y);
    }

    float sinc(float x) const {
        x = std::abs(x);
        if (x < 1e-5) {
            return 1;
        }
        return std::sin(PI * x) / (PI * x);
    }

    float windowed_sinc(float x, float radius) const {
        x = std::abs(x);
        if (x > radius) {
            return 0;
        }
        float lanczos = sinc(x / tau_);
        return sinc(x) * lanczos;
    }

private:
    float tau_;
};

} // namespace turner
