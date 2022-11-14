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
    typedef std::chrono::time_point<std::chrono::system_clock> TimePoint;
    typedef std::chrono::duration<double> Duration;

    static TimePoint getCurrentTimePoint();
    static void sleepForMilliSeconds(const long long milliSec);

private:
    TimePoint start;
};

}  // namespace lenny::tools