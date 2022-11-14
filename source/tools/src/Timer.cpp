#include <lenny/tools/Timer.h>

#include <thread>

namespace lenny::tools {

void Timer::restart() {
    start = getCurrentTimePoint();
}

double Timer::time() const {
    const TimePoint now = getCurrentTimePoint();
    Duration duration = now - start;
    return duration.count();
}

void Timer::sleep(const double time, bool waitAccurately) {
    if (waitAccurately) {
        const TimePoint begin = getCurrentTimePoint();
        Duration duration = begin - begin;
        while (duration.count() < time)
            duration = getCurrentTimePoint() - begin;
    } else {
        sleepForMilliSeconds((long long)(time * 1000.0));
    }
}

Timer::TimePoint Timer::getCurrentTimePoint() {
    return std::chrono::high_resolution_clock::now();
}

void Timer::sleepForMilliSeconds(const long long milliSec) {
    std::this_thread::sleep_for(std::chrono::duration(std::chrono::milliseconds(milliSec)));
}

}  // namespace lenny::tools