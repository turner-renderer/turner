#pragma once

#include <chrono>
#include <functional>
#include <ostream>
#include <unordered_map>

namespace turner {

/**
 * Profiler
 *
 * Internally profiler uses system signals to make samples. Technically it is a global object.
 * Therefore, we provide here a C API.
 *
 * Each start call should be accompanied by a stop call. Cf. `Profile` to enable profiling for a
 * specific category.
 */
void profiler_start();
void profiler_clear();
std::chrono::seconds profiler_stop();

enum class ProfCategory { Render, Trace, Intersect, SamplingHemisphere, size };
static const char* ProfCategoryNames[]{"render", "trace()",
                                       "KDTree::intersect()",
                                       "sampling::hemisphere()", "total"};
static_assert(static_cast<size_t>(ProfCategory::size) <= 64,
              "too many profiling categories");

/**
 * Profile using sampling approach.
 *
 * The samples will be accumulated for the category set in the constructor.
 *
 * Usage:
 *
 * ```
 * some_block_to_profile {
 *     Profile _(ProfCategory::SomeCategory);
 *     // ... to work
 * }
 * ```
 */
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
