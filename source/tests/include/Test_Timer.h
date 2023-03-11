#pragma once

#include <lenny/tools/Timer.h>

#include <iostream>

void test_timer() {
    lenny::tools::Timer timer;

    double sleepTime = 0.00001;
    for (int i = 0; i <= 5; i++) {
        std::cout << "Sleep time: " << sleepTime << std::endl;

        timer.restart();
        timer.sleep(sleepTime, true);
        std::cout << "ERROR ACCURATE: " << timer.time() - sleepTime << std::endl;

        timer.restart();
        timer.sleep(sleepTime, false);
        std::cout << "ERROR INACCURATE: " << timer.time() - sleepTime << std::endl;

        sleepTime *= 10.0;
    }
}