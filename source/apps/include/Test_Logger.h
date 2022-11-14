#pragma once

#include <lenny/tools/Logger.h>

#include <thread>

void log_test(int n) {
    LENNY_LOG_INFO("%d", n);
    LENNY_LOG_WARNING("%d", n);
}

void test_logger() {
    using lenny::tools::Logger;
    std::thread threads[30];

    for (int i = 0; i < 30; i++)
        threads[i] = std::thread(log_test, i);

    for (int i = 0; i < 30; i++)
        threads[i].join();

    std::cout << "-------------------------- BUFFER TEST -------------------------------" << std::endl;
    const std::vector<std::pair<Logger::COLOR, std::string>>& msgBuffer = Logger::getMessageBuffer();
    for (const auto& [color, msg] : msgBuffer)
        std::cout << Logger::getColorString(color) << msg << Logger::getColorString(Logger::DEFAULT);
}