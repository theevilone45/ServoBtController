#pragma once

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <functional>
#include "Config.hpp"
#include "Logger.hpp"
#include "MessageProtocol.hpp"

// Forward declaration
void fatalError(const String& errorMessage);

class BluetoothService
{
  public:
    static constexpr int32_t CRITICAL_BLUETOOTH_ERROR = 1000;
    enum class BluetoothMessageResult : int32_t
    {
        OK = 0,
        ROBOT_BUSY,
        VALIDATION_FAILED,
        SERVO_UNINITIALIZED = CRITICAL_BLUETOOTH_ERROR,
        SENDING_MESSAGE_FAILED,
        PROCESSING_FAILED
    };

    using ProcessingCallbackType = std::function<BluetoothMessageResult(const RawMessage&)>;

    BluetoothService(ProcessingCallbackType&& processingCallback);
    
    bool isCriticalError(BluetoothMessageResult result) const;
    void runStep();
    [[nodiscard]] bool isConnected() const;
    [[nodiscard]] String receive();
    [[nodiscard]] bool send(const String& message);
    [[nodiscard]] bool sendAckResponse(uint16_t msg_id);
    [[nodiscard]] bool sendTaskFinishedResponse(uint16_t msg_id);

  private:
    [[nodiscard]] BLEDevice getCentralDevice() const;

    const ProcessingCallbackType mProcessingCallback;
    BLEService mTextService;
    BLEStringCharacteristic mMessageService;
};
