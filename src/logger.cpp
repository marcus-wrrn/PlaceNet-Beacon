#include "logger.h"
#include <stdarg.h>

log_level_t Logger::currentLevel = LOG_LEVEL_INFO;

void Logger::setLogLevel(log_level_t level) {
    currentLevel = level;
}

log_level_t Logger::getLogLevel() {
    return currentLevel;
}

const char* Logger::getLevelString(log_level_t level) {
    switch (level) {
        case LOG_LEVEL_ERROR:   return "E";
        case LOG_LEVEL_WARN:    return "W";
        case LOG_LEVEL_INFO:    return "I";
        case LOG_LEVEL_DEBUG:   return "D";
        case LOG_LEVEL_VERBOSE: return "V";
        default:                return "?";
    }
}

const char* Logger::getLevelColor(log_level_t level) {
    switch (level) {
        case LOG_LEVEL_ERROR:   return "\033[0;31m";
        case LOG_LEVEL_WARN:    return "\033[0;33m";
        case LOG_LEVEL_INFO:    return "\033[0;32m";
        case LOG_LEVEL_DEBUG:   return "\033[0;36m";
        case LOG_LEVEL_VERBOSE: return "\033[0;37m";
        default:                return "\033[0m";
    }
}

void Logger::log(log_level_t level, const char* tag, const char* format, va_list args) {
    if (level > currentLevel) {
        return;
    }

    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);

    Serial.printf("%s%s (%lu) [%s]: %s\033[0m\n",
                  getLevelColor(level),
                  getLevelString(level),
                  millis(),
                  tag,
                  buffer);
}

void Logger::logError(const char* tag, const char* format, ...) {
    va_list args;
    va_start(args, format);
    log(LOG_LEVEL_ERROR, tag, format, args);
    va_end(args);
}

void Logger::logWarn(const char* tag, const char* format, ...) {
    va_list args;
    va_start(args, format);
    log(LOG_LEVEL_WARN, tag, format, args);
    va_end(args);
}

void Logger::logInfo(const char* tag, const char* format, ...) {
    va_list args;
    va_start(args, format);
    log(LOG_LEVEL_INFO, tag, format, args);
    va_end(args);
}

void Logger::logDebug(const char* tag, const char* format, ...) {
    va_list args;
    va_start(args, format);
    log(LOG_LEVEL_DEBUG, tag, format, args);
    va_end(args);
}

void Logger::logVerbose(const char* tag, const char* format, ...) {
    va_list args;
    va_start(args, format);
    log(LOG_LEVEL_VERBOSE, tag, format, args);
    va_end(args);
}
