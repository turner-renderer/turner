#include <chrono>
#include <ostream>
#include <string>


class Runtime {
public:
    Runtime(std::ostream& os, const std::string& label)
        : os_(os)
        , label_(label)
        , started_(std::chrono::steady_clock::now())
        {}
    ~Runtime() {
        auto now = std::chrono::steady_clock::now();
        os_ << label_ << std::chrono::duration_cast<std::chrono::milliseconds>(
            now - started_).count()
            << " ms" << std::endl;
    }

private:
    std::ostream& os_;
    std::string label_;
    std::chrono::steady_clock::time_point started_;
};
