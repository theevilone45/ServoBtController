#pragma once

#include <Arduino.h>
#include "Config.hpp"

enum class ModuleId
{
    BLE = 0,
    SERVO,
};

enum class Severity
{
    PERIODIC = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4
};

const char* getModuleString(ModuleId module);
const char* getSeverityString(Severity severity);
void initLogger();

template <ModuleId MODULE, Severity LEVEL> class Logger
{
  public:
    static void log(const String& message)
    {
        if (static_cast<int>(LEVEL) >= MIN_LOG_LEVEL)
        {
            Serial.print(getModuleString(MODULE));
            Serial.print(" ");
            Serial.print(getSeverityString(LEVEL));
            Serial.print(": ");
            Serial.println(message);
        }
    }
};

// Logger type aliases for BLE
using BlePeriodicLogger = Logger<ModuleId::BLE, Severity::PERIODIC>;
using BleDebugLogger = Logger<ModuleId::BLE, Severity::DEBUG>;
using BleInfoLogger = Logger<ModuleId::BLE, Severity::INFO>;
using BleWarnLogger = Logger<ModuleId::BLE, Severity::WARN>;
using BleErrorLogger = Logger<ModuleId::BLE, Severity::ERROR>;

// Logger type aliases for Servo
using ServoPeriodicLogger = Logger<ModuleId::SERVO, Severity::PERIODIC>;
using ServoDebugLogger = Logger<ModuleId::SERVO, Severity::DEBUG>;
using ServoInfoLogger = Logger<ModuleId::SERVO, Severity::INFO>;
using ServoWarnLogger = Logger<ModuleId::SERVO, Severity::WARN>;
using ServoErrorLogger = Logger<ModuleId::SERVO, Severity::ERROR>;

// Logging macros for BLE
#define LOG_PERIODIC_BLE(msg) BlePeriodicLogger::log(msg)
#define LOG_DEBUG_BLE(msg) BleDebugLogger::log(msg)
#define LOG_INFO_BLE(msg) BleInfoLogger::log(msg)
#define LOG_WARN_BLE(msg) BleWarnLogger::log(msg)
#define LOG_ERROR_BLE(msg) BleErrorLogger::log(msg)

// Logging macros for Servo
#define LOG_PERIODIC_SERVO(msg) ServoPeriodicLogger::log(msg)
#define LOG_DEBUG_SERVO(msg) ServoDebugLogger::log(msg)
#define LOG_INFO_SERVO(msg) ServoInfoLogger::log(msg)
#define LOG_WARN_SERVO(msg) ServoWarnLogger::log(msg)
#define LOG_ERROR_SERVO(msg) ServoErrorLogger::log(msg)
