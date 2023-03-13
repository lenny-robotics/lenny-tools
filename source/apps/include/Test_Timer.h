#pragma once

#include <gtest/gtest.h>
#include <lenny/tools/Timer.h>

#include <iostream>

TEST(tools, Timer) {
    lenny::tools::Timer timer;

    double sleepTime = 0.00001;
    for (int i = 0; i <= 5; i++) {
        std::cout << "Sleep time: " << sleepTime << std::endl;

        timer.restart();
        timer.sleep(sleepTime, true);
        //EXPECT_LE(timer.time() - sleepTime, 1e-5);
        std::cout << "ERROR ACCURATE: " << timer.time() - sleepTime << std::endl;

        timer.restart();
        timer.sleep(sleepTime, false);
        //EXPECT_LE(timer.time() - sleepTime, 1e-3);
        std::cout << "ERROR INACCURATE: " << timer.time() - sleepTime << std::endl;

        sleepTime *= 10.0;
    }
}