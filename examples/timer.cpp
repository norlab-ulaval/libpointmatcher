#include "pointmatcher/Timer.h"
#include <iostream>
#include <chrono>
#include <thread>

namespace Time {
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;
}

int main() {
    const Time::TimePoint time1 = Time::Clock::now();
    PointMatcherSupport::timer t1;

    std::this_thread::sleep_for(std::chrono::seconds(10));

    const Time::TimePoint time2 = Time::Clock::now();
    const double elapsed = t1.elapsed();

    std::cout << "PM:timer time = " << elapsed << "\nchrono duration = "
              << std::chrono::duration_cast<std::chrono::nanoseconds>(time2 - time1).count() / double(1000000000)
              << " ms" << std::endl;

    return 0;
}
