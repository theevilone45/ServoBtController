#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include "Config.hpp"
#include "Logger.hpp"
#include "MessageProtocol.hpp"

struct ServoConstaint
{
    const int MIN_POSITION;
    const int MAX_POSITION;
};

enum class ServoState
{
    IDLE = 0,
    MOVING
};

struct ServoDevice
{
    const int pin;
    Servo servo;
    ServoState state;
    int currentPosition;
    int targetPosition;
    const int delay;
    const ServoConstaint constraints;
};

class ServoService
{
  public:
    enum class ServoActionResult
    {
        IN_PROGRESS = 0,
        OUT_OF_RANGE,
        UNKNOWN_FAILURE
    };

    ServoService();
    
    [[nodiscard]] bool initServos();
    void updateServos();
    [[nodiscard]] ServoActionResult moveServos(int16_t verticalOffset, int16_t horizontalOffset);
    [[nodiscard]] bool parseServoRequest(const RawMessage& rawMessage, ServoRequest& command);
    [[nodiscard]] ServoState getVerticalServoState() const;
    [[nodiscard]] ServoState getHorizontalServoState() const;
    [[nodiscard]] bool areServosIdle() const;
    [[nodiscard]] uint16_t getCurrentMsgId() const;
    void setCurrentMsgId(uint16_t msgId);

  private:
    [[nodiscard]] int getHorizontalOffset() const;
    [[nodiscard]] int getVerticalOffset() const;
    void updateHorizontal(unsigned long currentMillis);
    void updateVertical(unsigned long currentMillis);

    ServoDevice mHorizontalServoDevice;
    ServoDevice mVerticalServoDevice;
    unsigned long mLastHorizontalUpdateTime = 0;
    unsigned long mLastVerticalUpdateTime = 0;
    uint16_t mCurrentMsgId = 0;
};
