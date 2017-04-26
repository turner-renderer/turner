#include "profile.h"

#include <signal.h>
#include <sys/time.h>

#include <array>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <string>

namespace turner {

namespace {

struct ProfilerSample {
    uint64_t state = 0; // hash table key
    size_t count = 0;   // hash table value
};

__thread uint64_t profiler_state;
static std::atomic<bool> profiler_running{false};
static std::chrono::steady_clock::time_point profiler_start_time;

static constexpr int profiler_samples_size = 256;
// static size hash table (cf. `get_profiler_samples_index`)
static std::array<ProfilerSample, profiler_samples_size> profiler_samples;

/**
 * Return the index of the bucket in the hash table.
 *
 * The collision resolution strategy is to go to the next bucket on collision.
 *
 * @param  state key
 * @return       index of the bucket in the hash table.
 */
size_t get_profiler_samples_index(uint64_t state) {
    uint64_t i = std::hash<uint64_t>{}(state) % (profiler_samples_size - 1);
    int num_visited = 0;
    while (num_visited < profiler_samples_size &&
           profiler_samples[i].state != state &&
           profiler_samples[i].state != 0) {
        ++i;
        if (i == profiler_samples_size) {
            i = 0;
        }
        ++num_visited;
    }
    assert(num_visited < profiler_samples_size &&
           "Profiler samples hash table too small.");

    return i;
}

/**
 * Signal callback for sampling profiler and reporting sample.
 */
void profiler_make_sample(int, siginfo_t*, void*) {
    if (profiler_state == 0) {
        return;
    }

    uint64_t i = get_profiler_samples_index(profiler_state);
    profiler_samples[i].state = profiler_state;
    ++profiler_samples[i].count;
}

} // namespace

void profiler_start() {
    assert(!profiler_running && "logic error");

    profiler_start_time = std::chrono::steady_clock::now();

    // profile signal
    struct sigaction sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = profiler_make_sample;
    sa.sa_flags = SA_RESTART | SA_SIGINFO;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGPROF, &sa, NULL);

    static struct itimerval timer;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 1000000 / 100; // 100 Hz sampling
    timer.it_value = timer.it_interval;

    if (setitimer(ITIMER_PROF, &timer, NULL) != 0) {
        throw std::runtime_error(std::string("Could not initialize timer: ") +
                                 strerror(errno));
    }

    profiler_running = true;
}

void profiler_clear() {
    for (auto& ps : profiler_samples) {
        ps.state = 0;
        ps.count = 0;
    }
}

std::chrono::seconds profiler_stop() {
    assert(profiler_running);
    auto profiler_runtime = std::chrono::steady_clock::now() - profiler_start_time;

    static struct itimerval timer;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 0;
    timer.it_value = timer.it_interval;

    if (setitimer(ITIMER_PROF, &timer, NULL) != 0) {
        throw std::runtime_error(std::string("Failed to disable timer: ") +
                                 strerror(errno));
    }

    profiler_running = false;
    return std::chrono::duration_cast<std::chrono::seconds>(profiler_runtime);
}

Profile::Profile(ProfCategory category) : bit_(static_cast<size_t>(category)) {
    profiler_state |= (1ull << bit_);
}
Profile::~Profile() { profiler_state &= ~(1ull << bit_); }

ProfilerResults profiler_get_results() {
    std::unordered_map<ProfCategory, size_t> data;
    for (const auto& ps : profiler_samples) {
        // Use `size` of categories to store the total number of samples.
        data[ProfCategory::size] += ps.count;
        for (size_t bit = 0; bit < 64; ++bit) {
            if (ps.state & (1ull << bit)) {
                data[static_cast<ProfCategory>(bit)] += ps.count;
            }
        }
    }
    return {data};
}

std::ostream& operator<<(std::ostream& os, const ProfilerResults& res) {
    auto it = res.category_counts.find(ProfCategory::size);
    assert(it != res.category_counts.end());
    double total = it->second;

    for (const auto& kv : res.category_counts) {
        if (kv.first == ProfCategory::size) {
            continue;
        }

        const char* category_name = ProfCategoryNames[kv.first];
        const double category_percent = (100. * kv.second / total);

        os << std::setw(20) << std::setfill(' ') << std::left << category_name
           << std::setw(7) << std::setfill(' ') << std::right << std::fixed
           << std::setprecision(2) << category_percent << '%';
    }
    return os;
}
} // namespace turner
