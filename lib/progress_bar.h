#pragma once

#include <iomanip>
#include <cmath>
#include <ostream>
#include <string>

class ProgressBar {

public:
    explicit ProgressBar(std::ostream& out, const std::string label, const size_t max)
        : out_(out), label_(label), max_(max) {}

    void update(const size_t current) const {
        const float progress = static_cast<float>(current) / max_;
        const int bar_width = std::min(static_cast<int>(progress * 20), 20);
        out_ << "\r"
             << std::setw(20) << std::setfill(' ') << std::left << label_
             << "[" << std::string(bar_width, '-')
             << std::string(20 - bar_width, ' ') << "]"
             << std::setw(7) << std::setfill(' ') << std::right
             << std::fixed << std::setprecision(2) << (progress * 100.0) << '%';
        out_.flush();
    }
private:
    std::ostream& out_;
    const std::string label_;
    const size_t max_;
};
