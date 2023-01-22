#include <lenny/tools/Logger.h>

#if WIN32
#include <windows.h>
#endif

namespace lenny::tools {

Logger::PRIORITY Logger::priority = Logger::DEBUG;
bool Logger::logToFile = true;
std::string Logger::logFilePath = LENNY_PROJECT_FOLDER "/logs/logs.txt";
bool Logger::logToBuffer = true;
int Logger::maxNumBufferMsgs = 20;

std::array<std::string, 7> Logger::colorList = {"\x1b[0m", "\x1b[31m", "\x1b[32m", "\x1b[33m", "\x1b[34m", "\x1b[35m", "\x1b[36m"};
std::array<std::array<double, 3>, 7> Logger::rgbList = {
    std::array<double, 3>{1.0, 1.0, 1.0}, std::array<double, 3>{1.0, 0.0, 0.0}, std::array<double, 3>{0.0, 1.0, 0.0}, std::array<double, 3>{1.0, 1.0, 0.0},
    std::array<double, 3>{0.0, 0.0, 1.0}, std::array<double, 3>{1.0, 0.0, 1.0}, std::array<double, 3>{0.0, 1.0, 1.0}};
std::mutex Logger::logMutex;
FILE* Logger::file = nullptr;
std::deque<std::pair<Logger::COLOR, std::string>> Logger::msgBuffer = {};

void Logger::initialize() {
    //Initialize system, so colors get displayed properly (function is called only once)
#if WIN32
    static bool initializeSystem = []() {
        system(("chcp " + std::to_string(CP_UTF8)).c_str());
        return true;
    }();
#endif
}

std::string Logger::getColorString(COLOR color) {
    return colorList[color];
}

std::array<double, 3> Logger::getColorArray(COLOR color) {
    return rgbList[color];
}

const std::deque<std::pair<Logger::COLOR, std::string>>& Logger::getMessageBuffer() {
    return msgBuffer;
}

}  // namespace lenny::tools