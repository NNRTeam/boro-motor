#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <Config.h>

class Logger
{
public:
    enum Level {
        DEBUG = 0,
        INFO = 1,
        WARN = 2,
        ERROR = 3,
        NONE = 4
    };

    Logger(Level level = INFO) : logLevel(level) {}

    void setLogLevel(Level level) {
        logLevel = level;
    }

    void debug(const String &message) {
        log(DEBUG, "DEBUG: " + message);
    }

    void info(const String &message) {
        log(INFO, "INFO: " + message);
    }

    void warn(const String &message) {
        log(WARN, "WARN: " + message);
    }

    void error(const String &message) {
        log(ERROR, "ERROR: " + message);
    }
private:
    Level logLevel;

    void log(Level level, const String &message) {
        if (config::ENABLE_SERIAL_DEBUG && level >= logLevel) {
            Serial.println(message);
        }
    }
};

#endif // LOGGER_H