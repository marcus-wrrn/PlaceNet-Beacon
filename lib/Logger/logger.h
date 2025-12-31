#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

typedef enum {
    LOG_LEVEL_NONE,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_VERBOSE
} log_level_t;

class Logger {
public:
    static void setLogLevel(log_level_t level);
    static log_level_t getLogLevel();

    static void logError(const char* tag, const char* format, ...);
    static void logWarn(const char* tag, const char* format, ...);
    static void logInfo(const char* tag, const char* format, ...);
    static void logDebug(const char* tag, const char* format, ...);
    static void logVerbose(const char* tag, const char* format, ...);

private:
    static log_level_t currentLevel;
    static void log(log_level_t level, const char* tag, const char* format, va_list args);
    static const char* getLevelString(log_level_t level);
};

#define LOG_ERROR(tag, format, ...)   Logger::logError(tag, format, ##__VA_ARGS__)
#define LOG_WARN(tag, format, ...)    Logger::logWarn(tag, format, ##__VA_ARGS__)
#define LOG_INFO(tag, format, ...)    Logger::logInfo(tag, format, ##__VA_ARGS__)
#define LOG_DEBUG(tag, format, ...)   Logger::logDebug(tag, format, ##__VA_ARGS__)
#define LOG_VERBOSE(tag, format, ...) Logger::logVerbose(tag, format, ##__VA_ARGS__)

#define LOGE(tag, format, ...) LOG_ERROR(tag, format, ##__VA_ARGS__)
#define LOGW(tag, format, ...) LOG_WARN(tag, format, ##__VA_ARGS__)
#define LOGI(tag, format, ...) LOG_INFO(tag, format, ##__VA_ARGS__)
#define LOGD(tag, format, ...) LOG_DEBUG(tag, format, ##__VA_ARGS__)
#define LOGV(tag, format, ...) LOG_VERBOSE(tag, format, ##__VA_ARGS__)

#endif
