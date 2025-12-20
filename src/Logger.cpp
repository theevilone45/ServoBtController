#include "Logger.hpp"

const char* getModuleString(ModuleId module)
{
    switch (module)
    {
    case ModuleId::BLE:
        return "[BLE]";
    case ModuleId::SERVO:
        return "[SERVO]";
    default:
        return "[UNKNOWN]";
    }
}

const char* getSeverityString(Severity severity)
{
    switch (severity)
    {
    case Severity::PERIODIC:
        return "[PERIODIC]";
    case Severity::DEBUG:
        return "[DEBUG]";
    case Severity::INFO:
        return "[INFO]";
    case Severity::WARN:
        return "[WARN]";
    case Severity::ERROR:
        return "[ERROR]";
    default:
        return "[UNKNOWN]";
    }
}

void initLogger()
{
    Serial.begin(9600);
    while (!Serial)
        ;
}
