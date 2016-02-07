#include <chrono>
#include <ostream>
#include <string>


class Runtime {
public:
    explicit Runtime(size_t& result_ms)
        : started_(std::chrono::steady_clock::now())
        , result_ms_(result_ms)
        {}
    ~Runtime() {
        auto now = std::chrono::steady_clock::now();
        result_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - started_).count();
    }

private:
    std::chrono::steady_clock::time_point started_;
    size_t& result_ms_;
};
