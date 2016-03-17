#include <chrono>
#include <ostream>
#include <string>


class Runtime {
public:
    Runtime() {}
    explicit Runtime(size_t& result_ms)
        : result_ms_(&result_ms)
        {}

    // return runtime in ms
    size_t operator()() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            now - started_).count();
    }

    ~Runtime() {
        if (result_ms_) {
            *result_ms_ = (*this)();
        }
    }

private:
    std::chrono::steady_clock::time_point started_ =
        std::chrono::steady_clock::now();
    size_t* result_ms_ = nullptr;
};
