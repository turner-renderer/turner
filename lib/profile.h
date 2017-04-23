#pragma once

#include <unordered_map>

namespace turner {

void profiler_init();
void profiler_clear();
void profiler_destroy();

enum class ProfCategory { cat1, cat2, cat3, size };
static const char* ProfCategoryNames[]{"cat1", "cat2", "cat3", "total"};
static_assert(static_cast<size_t>(ProfCategory::size) <= 64,
              "too many profiling categories");

std::unordered_map<ProfCategory, size_t> profiler_results();

class Profile {
public:
    Profile(ProfCategory category);
    ~Profile();

    Profile(const Profile&) = delete;
    Profile& operator=(const Profile&) = delete;

private:
    size_t bit_;
};
} // namespace turner

namespace std {
template <> struct hash<turner::ProfCategory> {
    size_t operator()(const turner::ProfCategory& cat) const {
        return std::hash<uint64_t>{}(static_cast<uint64_t>(cat));
    }
};
} // namespace std
