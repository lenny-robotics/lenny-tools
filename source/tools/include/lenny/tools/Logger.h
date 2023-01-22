#pragma once
#if !defined WIN32
#pragma GCC diagnostic ignored "-Wformat-security"
#endif

#include <lenny/tools/Utils.h>

#include <array>
#include <deque>
#include <magic_enum.hpp>
#include <memory>
#include <mutex>

namespace lenny::tools {

class Logger {
private:  //Make private, since this is a purely static class
    Logger() = default;
    ~Logger() = default;

public:
    //--- Members
    enum COLOR { DEFAULT, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN };
    enum PRIORITY { PRINT, ERROR, WARNING, INFO, DEBUG };

private:
    //--- Private helpers
    static void initialize();

    template <typename... Args>
    static std::string getString(const char* format, Args... args) {
        int size_s = std::snprintf(nullptr, 0, format, args...) + 1;  // Extra space for '\0'
        if (size_s <= 0)
            return std::string("");
        const size_t size = static_cast<size_t>(size_s);
        std::unique_ptr<char[]> buf(new char[size]);
        std::snprintf(buf.get(), size, format, args...);
        return std::string(buf.get(), buf.get() + size - 1);  // We don't want the '\0' inside
    }

    template <typename... Args>
    static void print(bool printExtendedInfo, PRIORITY prio, COLOR color, const char* function, const char* message, Args... args) {
        //Initialize
        initialize();

        //Lock scope
        std::scoped_lock lock(logMutex);

        //Log to terminal
        if (prio <= priority) {
            printf("%s", colorList[color].c_str());
            if (printExtendedInfo) {
                printf("[LENNY](%s) in ", std::string(magic_enum::enum_name(prio)).c_str());
                printf("'%s' at ", function);
                printf("%s: ", utils::getCurrentDateAndTime().c_str());
            }
            printf("%s", getString(message, args...).c_str());
            if (printExtendedInfo)
                printf("\n");
            printf("%s", colorList[DEFAULT].c_str());
        }

        //Log to buffer
        if (logToBuffer && prio <= priority)
            printToBuffer(printExtendedInfo, prio, color, function, message, args...);

        //Always print all infos to file, no matter the priority
        if (logToFile)
            printToFile(printExtendedInfo, prio, color, function, message, args...);
    }

    template <typename... Args>
    static void printToFile(bool printExtendedInfo, PRIORITY prio, COLOR color, const char* function, const char* message, Args... args) {
        if (!file) {
            file = fopen(logFilePath.c_str(), "wt");
            if (!file)
                printf("%sWARNING in Logger::printToFile: Failed to open log file at `%s`%s", colorList[YELLOW].c_str(), logFilePath.c_str(),
                       colorList[DEFAULT].c_str());
        }
        if (file) {
            if (printExtendedInfo) {
                fprintf(file, "[LENNY](%s) in ", std::string(magic_enum::enum_name(prio)).c_str());
                fprintf(file, "'%s' at ", function);
                fprintf(file, "%s: ", utils::getCurrentDateAndTime().c_str());
            }
            fprintf(file, "%s", getString(message, args...).c_str());
            if (printExtendedInfo)
                fprintf(file, "\n");
        }
    }

    template <typename... Args>
    static void printToBuffer(bool printExtendedInfo, PRIORITY prio, COLOR color, const char* function, const char* message, Args... args) {
        std::string msg;
        if (printExtendedInfo) {
            msg += getString("[LENNY](%s) in ", std::string(magic_enum::enum_name(prio)).c_str());
            msg += getString("'%s' at ", function);
            msg += getString("%s: ", utils::getCurrentDateAndTime().c_str());
        }
        msg += getString(message, args...);
        if (printExtendedInfo)
            msg += getString("\n");
        msgBuffer.push_back(std::pair<COLOR, std::string>{color, msg});
        if (msgBuffer.size() > maxNumBufferMsgs)
            msgBuffer.pop_front();
    }

public:
    //--- Logs
    template <typename... Args>
    static void print(COLOR color, const char* message, Args... args) {
        print(false, PRINT, color, "", message, args...);
    }

    template <typename... Args>
    static void error(const char* function, const char* message, Args... args) {
        print(true, ERROR, RED, function, message, args...);
        exit(0);
    }

    template <typename... Args>
    static void warning(const char* function, const char* message, Args... args) {
        print(true, WARNING, YELLOW, function, message, args...);
    }

    template <typename... Args>
    static void info(const char* function, const char* message, Args... args) {
        print(true, INFO, GREEN, function, message, args...);
    }

    template <typename... Args>
    static void debug(const char* function, const char* message, Args... args) {
        print(true, DEBUG, BLUE, function, message, args...);
    }

    //--- Public helpers
    static std::string getColorString(COLOR color);
    static std::array<double, 3> getColorArray(COLOR color);
    static const std::deque<std::pair<COLOR, std::string>>& getMessageBuffer();

public:
    //--- Public members
    static PRIORITY priority;
    static bool logToFile;
    static std::string logFilePath;
    static bool logToBuffer;
    static int maxNumBufferMsgs;

private:
    //--- Private members
    static std::array<std::string, 7> colorList;
    static std::array<std::array<double, 3>, 7> rgbList;
    static std::mutex logMutex;
    static FILE* file;
    static std::deque<std::pair<COLOR, std::string>> msgBuffer;
};

}  // namespace lenny::tools

//--- Macros
#ifdef NDEBUG
#define LENNY_FUNCTION __FUNCTION__
#else
#if WIN32
#define LENNY_FUNCTION __FUNCSIG__
#else
#define LENNY_FUNCTION __PRETTY_FUNCTION__
#endif
#endif

#define LENNY_LOG_PRINT(color, msg, ...) lenny::tools::Logger::print(color, msg, ##__VA_ARGS__);
#define LENNY_LOG_ERROR(msg, ...) lenny::tools::Logger::error(LENNY_FUNCTION, msg, ##__VA_ARGS__);
#define LENNY_LOG_WARNING(msg, ...) lenny::tools::Logger::warning(LENNY_FUNCTION, msg, ##__VA_ARGS__);
#define LENNY_LOG_INFO(msg, ...) lenny::tools::Logger::info(LENNY_FUNCTION, msg, ##__VA_ARGS__);
#define LENNY_LOG_DEBUG(msg, ...) lenny::tools::Logger::debug(LENNY_FUNCTION, msg, ##__VA_ARGS__);