#pragma once

// BLE Configuration
constexpr const char* cTextServiceUuid = "12345678-1234-1234-1234-1234567890ab";
constexpr const char* cMessageServiceUuid = "abcd1234-1234-1234-1234-1234567890ab";
constexpr const char* cLocalName = "ArduinoBL";

// Servo Configuration
constexpr const int cVerticalServoPin = 5;
constexpr const int cHorizontalServoPin = 4;
constexpr const int cVerticalServoDelayMs = 1;
constexpr const int cHorizontalServoDelayMs = 1;

// Logging Configuration
#ifndef MIN_LOG_LEVEL
#define MIN_LOG_LEVEL 2 // 0 - PERIODIC, 1 - DEBUG, 2 - INFO, 3 - WARN, 4 - ERROR
#endif
