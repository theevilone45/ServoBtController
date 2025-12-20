#include "ServoService.hpp"

ServoService::ServoService()
    // clang-format off
    : mHorizontalServoDevice{
        cHorizontalServoPin, Servo(), ServoState::IDLE, 
        90, 90, cHorizontalServoDelayMs, ServoConstaint{0, 180} }
    , mVerticalServoDevice{ 
        cVerticalServoPin, Servo(), ServoState::IDLE, 
        90, 90, cVerticalServoDelayMs, ServoConstaint{45, 135} }
// clang-format on
{
    if (!initServos())
    {
        LOG_ERROR_SERVO("Failed to initialize servos");
    }
}

bool ServoService::initServos()
{
    LOG_INFO_SERVO("Initializing servos...");
    mVerticalServoDevice.servo.attach(mVerticalServoDevice.pin, 500, 2500);
    mHorizontalServoDevice.servo.attach(mHorizontalServoDevice.pin, 500, 2500);
    mVerticalServoDevice.servo.setPeriodHertz(50);
    mHorizontalServoDevice.servo.setPeriodHertz(50);
    LOG_INFO_SERVO("Servos initialized.");
    LOG_INFO_SERVO("Setting servos to default positions (90 degrees)...");
    mVerticalServoDevice.servo.write(90);
    mHorizontalServoDevice.servo.write(90);
    LOG_INFO_SERVO("Getting servos current positions...");
    mVerticalServoDevice.currentPosition = 90;
    mHorizontalServoDevice.currentPosition = 90;
    mVerticalServoDevice.targetPosition = 90;
    mHorizontalServoDevice.targetPosition = 90;
    LOG_INFO_SERVO(
        "Current positions - Vertical: " + String(mVerticalServoDevice.currentPosition) +
        ", Horizontal: " + String(mHorizontalServoDevice.currentPosition));
    delay(100); // Wait for servos to reach position
    return true;
}

void ServoService::updateServos()
{
    if (mVerticalServoDevice.state == ServoState::IDLE &&
        mHorizontalServoDevice.state == ServoState::IDLE)
    {
        return;
    }
    unsigned long currentMillis = millis();
    updateHorizontal(currentMillis);
    updateVertical(currentMillis);
}

ServoService::ServoActionResult ServoService::moveServos(int16_t verticalOffset, int16_t horizontalOffset)
{
    LOG_INFO_SERVO("Moving servos - Vertical offset: " + String(verticalOffset) +
                   ", Horizontal offset: " + String(horizontalOffset));
    int newVerticalPosition = mVerticalServoDevice.currentPosition + verticalOffset;
    int newHorizontalPosition = mHorizontalServoDevice.currentPosition + horizontalOffset;
    if (newVerticalPosition < mVerticalServoDevice.constraints.MIN_POSITION ||
        newVerticalPosition > mVerticalServoDevice.constraints.MAX_POSITION ||
        newHorizontalPosition < mHorizontalServoDevice.constraints.MIN_POSITION ||
        newHorizontalPosition > mHorizontalServoDevice.constraints.MAX_POSITION)
    {
        LOG_WARN_SERVO(
            "Requested servo positions out of range. Vertical: " + String(newVerticalPosition) +
            ", Horizontal: " + String(newHorizontalPosition));
        return ServoActionResult::OUT_OF_RANGE;
    }
    mVerticalServoDevice.targetPosition = newVerticalPosition;
    mHorizontalServoDevice.targetPosition = newHorizontalPosition;
    mVerticalServoDevice.state = ServoState::MOVING;
    mHorizontalServoDevice.state = ServoState::MOVING;
    LOG_INFO_SERVO("Servos moving to new positions.");
    return ServoActionResult::IN_PROGRESS;
}

bool ServoService::parseServoRequest(const RawMessage& rawMessage, ServoRequest& command)
{
    if (!decodeServoRequest(rawMessage, command))
    {
        LOG_ERROR_SERVO("Failed to decode incoming servo command");
        return false;
    }
    return true;
}

ServoState ServoService::getVerticalServoState() const
{
    return mVerticalServoDevice.state;
}

ServoState ServoService::getHorizontalServoState() const
{
    return mHorizontalServoDevice.state;
}

bool ServoService::areServosIdle() const
{
    return (mVerticalServoDevice.state == ServoState::IDLE) &&
           (mHorizontalServoDevice.state == ServoState::IDLE);
}

uint16_t ServoService::getCurrentMsgId() const
{
    return mCurrentMsgId;
}

void ServoService::setCurrentMsgId(uint16_t msgId)
{
    mCurrentMsgId = msgId;
}

int ServoService::getHorizontalOffset() const
{
    return mHorizontalServoDevice.targetPosition - mHorizontalServoDevice.currentPosition;
}

int ServoService::getVerticalOffset() const
{
    return mVerticalServoDevice.targetPosition - mVerticalServoDevice.currentPosition;
}

void ServoService::updateHorizontal(unsigned long currentMillis)
{
    if (currentMillis - mLastHorizontalUpdateTime >= mHorizontalServoDevice.delay)
    {
        const auto currentHorizontalOffset = getHorizontalOffset();
        if (currentHorizontalOffset != 0)
        {
            mHorizontalServoDevice.currentPosition += (currentHorizontalOffset > 0) ? 1 : -1;
            mHorizontalServoDevice.servo.write(mHorizontalServoDevice.currentPosition);
            LOG_DEBUG_SERVO("Horizontal servo moved to position: " +
                            String(mHorizontalServoDevice.currentPosition));
        }
        if (getHorizontalOffset() == 0)
        {
            mHorizontalServoDevice.state = ServoState::IDLE;
            LOG_PERIODIC_SERVO("Horizontal servo reached target position: " +
                           String(mHorizontalServoDevice.currentPosition));
        }
        mLastHorizontalUpdateTime = currentMillis;
    }
}

void ServoService::updateVertical(unsigned long currentMillis)
{
    if (currentMillis - mLastVerticalUpdateTime >= mVerticalServoDevice.delay)
    {
        const auto currentVerticalOffset = getVerticalOffset();
        if (currentVerticalOffset != 0)
        {
            mVerticalServoDevice.currentPosition += (currentVerticalOffset > 0) ? 1 : -1;
            mVerticalServoDevice.servo.write(mVerticalServoDevice.currentPosition);
            LOG_DEBUG_SERVO("Vertical servo moved to position: " +
                            String(mVerticalServoDevice.currentPosition));
        }
        if (getVerticalOffset() == 0)
        {
            mVerticalServoDevice.state = ServoState::IDLE;
            LOG_PERIODIC_SERVO("Vertical servo reached target position: " +
                           String(mVerticalServoDevice.currentPosition));
        }
        mLastVerticalUpdateTime = currentMillis;
    }
}
