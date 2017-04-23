#pragma once

#include <functional>
#include <ostream>
#include <unordered_map>

namespace turner {

void profiler_start();
void profiler_clear();
void profiler_stop();

enum class ProfCategory {
    Render,
    Trace,
    Intersect,
    IntersectInner,
    IntersectLeaf,
    SamplingHemisphere,
    size
};
static const char* ProfCategoryNames[]{"render",
                                       "trace()",
                                       "KDTree::intersect()",
                                       "KDTree::intersect()::inner",
                                       "KDTree::intersect()::leaf",
                                       "sampling::hemisphere()",
                                       "total"};
static_assert(static_cast<size_t>(ProfCategory::size) <= 64,
              "too many profiling categories");

class Profile {
public:
    Profile(ProfCategory category);
    ~Profile();

    Profile(const Profile&) = delete;
    Profile& operator=(const Profile&) = delete;

private:
    size_t bit_;
};

struct ProfilerResults {
    std::unordered_map<ProfCategory, size_t> category_counts;
};
ProfilerResults profiler_get_results();
std::ostream& operator<<(std::ostream& os, const ProfilerResults& res);

} // namespace turner
