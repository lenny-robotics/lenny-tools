#pragma once

#include <chrono>

namespace lenny::tools {

/**
 * Timer sleeps and reports time in seconds
 */
class Timer {
public:
    Timer() = default;
    ~Timer() = default;

    void restart();
    double time() const;
    static void sleep(const double time, bool waitAccurately = false);

private:
#if WIN32
    typedef std::chrono::steady_clock clock;
#else
    typedef std::chrono::system_clock clock;
#endif
    typedef std::chrono::time_point<clock> TimePoint;
    typedef std::chrono::duration<double> Duration;

    static TimePoint getCurrentTimePoint();
    static void sleepForMilliSeconds(const long long milliSec);

private:
    TimePoint start;
};

}  // namespace lenny::tools