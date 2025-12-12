#include <ArduinoBLE.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

#include <functional>
#include <memory>

namespace
{

constexpr const char* cTextServiceUuid = "12345678-1234-1234-1234-1234567890ab";
constexpr const char* cMessageServiceUuid = "abcd1234-1234-1234-1234-1234567890ab";
constexpr const char* cLocalName = "ArduinoBL";

constexpr const int cVerticalServoPin = 4;
constexpr const int cHorizontalServoPin = 5;
constexpr const int cServoDelayMs = 10;

#ifndef MIN_LOG_LEVEL
#define MIN_LOG_LEVEL 2 // 0 - PERIODIC, 1 - DEBUG, 2 - INFO, 3 - WARN, 4 - ERROR
#endif

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

using BlePeriodicLogger = Logger<ModuleId::BLE, Severity::PERIODIC>;
using BleDebugLogger = Logger<ModuleId::BLE, Severity::DEBUG>;
using BleInfoLogger = Logger<ModuleId::BLE, Severity::INFO>;
using BleWarnLogger = Logger<ModuleId::BLE, Severity::WARN>;
using BleErrorLogger = Logger<ModuleId::BLE, Severity::ERROR>;

using ServoPeriodicLogger = Logger<ModuleId::SERVO, Severity::PERIODIC>;
using ServoDebugLogger = Logger<ModuleId::SERVO, Severity::DEBUG>;
using ServoInfoLogger = Logger<ModuleId::SERVO, Severity::INFO>;
using ServoWarnLogger = Logger<ModuleId::SERVO, Severity::WARN>;
using ServoErrorLogger = Logger<ModuleId::SERVO, Severity::ERROR>;

} // namespace

#define LOG_PERIODIC_BLE(msg) BlePeriodicLogger::log(msg)
#define LOG_DEBUG_BLE(msg) BleDebugLogger::log(msg)
#define LOG_INFO_BLE(msg) BleInfoLogger::log(msg)
#define LOG_WARN_BLE(msg) BleWarnLogger::log(msg)
#define LOG_ERROR_BLE(msg) BleErrorLogger::log(msg)

#define LOG_PERIODIC_SERVO(msg) ServoPeriodicLogger::log(msg)
#define LOG_DEBUG_SERVO(msg) ServoDebugLogger::log(msg)
#define LOG_INFO_SERVO(msg) ServoInfoLogger::log(msg)
#define LOG_WARN_SERVO(msg) ServoWarnLogger::log(msg)
#define LOG_ERROR_SERVO(msg) ServoErrorLogger::log(msg)

using RawMessage = String;

struct ServoRequest
{
    uint16_t msg_id;
    int16_t s1_offset; // up / down
    int16_t s2_offset; // left / right
};

struct AckResponse
{
    uint16_t msg_id;
    bool success;
    String error_message;
};

struct TaskFinishedResponse
{
    uint16_t msg_id;
};

[[nodiscard]] bool decodeServoRequest(const String& jsonString, ServoRequest& outCommand)
{
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonString);
    if (error)
    {
        return false;
    }

    outCommand.msg_id = doc["msg_id"];
    outCommand.s1_offset = doc["s1_offset"];
    outCommand.s2_offset = doc["s2_offset"];

    return true;
}

[[nodiscard]] bool decodeAckResponse(const String& jsonString, AckResponse& outAck)
{
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonString);
    if (error)
    {
        return false;
    }

    outAck.msg_id = doc["msg_id"];
    outAck.success = doc["success"];
    outAck.error_message = doc["error_msg"].as<const char*>();

    return true;
}

[[nodiscard]] bool encodeServoRequest(const ServoRequest& command, String& outJsonString)
{
    JsonDocument doc;
    doc["msg_id"] = command.msg_id;
    doc["s1_offset"] = command.s1_offset;
    doc["s2_offset"] = command.s2_offset;

    serializeJson(doc, outJsonString);
    return true;
}

[[nodiscard]] bool encodeAckResponse(const AckResponse& ack, String& outJsonString)
{
    JsonDocument doc;
    doc["msg_id"] = ack.msg_id;
    doc["success"] = ack.success;
    doc["error_message"] = ack.error_message;

    serializeJson(doc, outJsonString);
    return true;
}

class BluetoothService
{
  public:
    enum class BluetoothMessageResult
    {
        OK = 0,
        VALIDATION_FAILED,
        PROCESSING_FAILED
    };

  public:
    using ProcessingCallbackType = std::function<BluetoothMessageResult(const RawMessage&)>;

  public:
    BluetoothService(ProcessingCallbackType&& processingCallback)
        : mProcessingCallback(std::move(processingCallback)), mTextService(cTextServiceUuid),
          mMessageService(cMessageServiceUuid,
                          BLERead | BLEWrite | BLEWriteWithoutResponse | BLENotify, 128)
    {
        LOG_INFO_BLE("Initializing BLE module...");
        if (!BLE.begin())
        {
            LOG_ERROR_BLE("Starting BLE module failed!");
            return;
        }
        else
        {
            LOG_INFO_BLE("BLE module started successfully");
        }

        if (!BLE.setLocalName(cLocalName))
        {
            LOG_ERROR_BLE("Failed to set local name");
            return;
        }
        else
        {
            LOG_DEBUG_BLE("Local name set to ArduinoBL");
        }

        if (!BLE.setAdvertisedService(mTextService))
        {
            LOG_ERROR_BLE("Failed to set advertised service");
            return;
        }
        else
        {
            LOG_DEBUG_BLE("Advertised service set");
        }

        mTextService.addCharacteristic(mMessageService);
        BLE.addService(mTextService);

        if (!BLE.advertise())
        {
            LOG_ERROR_BLE("Failed to start advertising");
            return;
        }
        else
        {
            LOG_DEBUG_BLE("Advertising started");
        }
    }

    void runStep()
    {
        LOG_DEBUG_BLE("Running BLE step");
        BLE.poll();
        String incoming = receive();
        if (incoming.length() > 0)
        {
            const auto processingResult = mProcessingCallback(incoming);
            if (processingResult != BluetoothMessageResult::OK)
            {
                LOG_ERROR_BLE("Error processing incoming message: raw data: " + incoming);
                return;
            }
            // ServoRequest command;
            // if (!decodeServoRequest(incoming, command))
            // {
            //     LOG_ERROR_BLE("Failed to decode incoming servo command");
            //     return;
            // }
            // const auto result = mProcessingCallback(command);
            // // Send acknowledgment
            // AckResponse ack;
            // ack.msg_id = command.msg_id;
            // ack.success = (result == ServoActionResult::OK);
            // if (!ack.success)
            // {
            //     ack.error_message =
            //         "Servo action failed with code: " + String(static_cast<int>(result));
            // }
            // LOG_DEBUG_BLE("Sending acknowledgment for msg_id: " + String(ack.msg_id));
            // String ackJson;
            // if (!encodeAckResponse(ack, ackJson))
            // {
            //     LOG_ERROR_BLE("Failed to encode acknowledgment message");
            //     return;
            // }
            // if (!send(ackJson))
            // {
            //     LOG_ERROR_BLE("Failed to send acknowledgment message");
            // }
        }
    }

    [[nodiscard]] bool isConnected() const
    {
        return getCentralDevice().connected();
    }

    [[nodiscard]] String receive()
    {
        if (mMessageService.written())
        {
            String incoming = mMessageService.value();
            LOG_INFO_BLE("Received: " + incoming);
            return incoming;
        }
        return String();
    }

    [[nodiscard]] bool send(const String& message)
    {
        LOG_INFO_BLE("Sending: " + message);
        return mMessageService.writeValue(message);
    }

  private:
    [[nodiscard]] BLEDevice getCentralDevice() const
    {
        return BLE.central();
    }

  private:
    const ProcessingCallbackType mProcessingCallback;
    BLEService mTextService;
    BLEStringCharacteristic mMessageService;
};

enum class ServoState
{
    IDLE = 0,
    MOVING
};

struct ServoDevice
{
    int pin;
    Servo servo;
    ServoState state;
    int currentPosition;
    int targetPosition;
};

class ServoService
{
  public:
    enum class ServoActionResult
    {
        OK = 0,
        OUT_OF_RANGE,
        UNKNOWN_FAILURE
    };

  public:
    ServoService()
        : mVerticalServoDevice{cVerticalServoPin, Servo(), ServoState::IDLE, 90, 90},
          mHorizontalServoDevice{cHorizontalServoPin, Servo(), ServoState::IDLE, 90, 90}
    {
        if (!initServos())
        {
            LOG_ERROR_SERVO("Failed to initialize servos");
        }
    }

    [[nodiscard]] bool initServos()
    {
        LOG_INFO_SERVO("Initializing servos...");
        mVerticalServoDevice.servo.attach(mVerticalServoDevice.pin, 500, 2400);
        mHorizontalServoDevice.servo.attach(mHorizontalServoDevice.pin, 500, 2400);
        mVerticalServoDevice.servo.setPeriodHertz(50);
        mHorizontalServoDevice.servo.setPeriodHertz(50);
        LOG_INFO_SERVO("Servos initialized.");
        // Set servos to known default positions first
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
        delay(1000); // Wait for servos to reach position
        return true;
    }

    void updateServos()
    {
        if (mVerticalServoDevice.state == ServoState::IDLE &&
            mHorizontalServoDevice.state == ServoState::IDLE)
        {
            return;
        }
        unsigned long currentMillis = millis();
        if (currentMillis - mLastUpdateTime >= mDelay)
        {
            bool verticalDone =
                (mVerticalServoDevice.currentPosition == mVerticalServoDevice.targetPosition);
            bool horizontalDone =
                (mHorizontalServoDevice.currentPosition == mHorizontalServoDevice.targetPosition);
            if (!verticalDone)
            {
                mVerticalServoDevice.currentPosition +=
                    (mVerticalServoDevice.targetPosition > mVerticalServoDevice.currentPosition)
                        ? 1
                        : -1;
                mVerticalServoDevice.servo.write(mVerticalServoDevice.currentPosition);
                LOG_DEBUG_SERVO("Vertical servo moved to position: " +
                                String(mVerticalServoDevice.currentPosition));
            }
            if (!horizontalDone)
            {
                mHorizontalServoDevice.currentPosition +=
                    (mHorizontalServoDevice.targetPosition > mHorizontalServoDevice.currentPosition)
                        ? 1
                        : -1;
                mHorizontalServoDevice.servo.write(mHorizontalServoDevice.currentPosition);
                LOG_DEBUG_SERVO("Horizontal servo moved to position: " +
                                String(mHorizontalServoDevice.currentPosition));
            }
            if (verticalDone)
            {
                mVerticalServoDevice.state = ServoState::IDLE;
                LOG_INFO_SERVO("Vertical servo reached target position: " +
                               String(mVerticalServoDevice.currentPosition));
            }
            if (horizontalDone)
            {
                mHorizontalServoDevice.state = ServoState::IDLE;
                LOG_INFO_SERVO("Horizontal servo reached target position: " +
                               String(mHorizontalServoDevice.currentPosition));
            }
            mLastUpdateTime = currentMillis;
        }
    }

    [[nodiscard]] ServoActionResult moveServos(int16_t verticalOffset, int16_t horizontalOffset)
    {
        LOG_INFO_SERVO("Moving servos - Vertical offset: " + String(verticalOffset) +
                       ", Horizontal offset: " + String(horizontalOffset));
        int newVerticalPosition = mVerticalServoDevice.currentPosition + verticalOffset;
        int newHorizontalPosition = mHorizontalServoDevice.currentPosition + horizontalOffset;
        if (newVerticalPosition < 0 || newVerticalPosition > 180 || newHorizontalPosition < 0 ||
            newHorizontalPosition > 180)
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
        return ServoActionResult::OK;
    }

    [[nodiscard]] bool parseServoRequest(const RawMessage& rawMessage, ServoRequest& command)
    {
        if (!decodeServoRequest(rawMessage, command))
        {
            LOG_ERROR_SERVO("Failed to decode incoming servo command");
            return false;
        }
        if (command.s1_offset > 180 || command.s2_offset > 180 || command.s1_offset < -180 ||
            command.s2_offset < -180)
        {
            LOG_WARN_SERVO("Received servo command with out of range offsets. Ignoring.");
            return false;
        }
        return true;
    }

    [[nodiscard]] ServoState getVerticalServoState() const
    {
        return mVerticalServoDevice.state;
    }

    [[nodiscard]] ServoState getHorizontalServoState() const
    {
        return mHorizontalServoDevice.state;
    }

  private:
    ServoDevice mVerticalServoDevice;
    ServoDevice mHorizontalServoDevice;
    const int mDelay = cServoDelayMs;
    unsigned long mLastUpdateTime = 0;
};

BluetoothService* gBleService;
ServoService* gServoService;

bool gRequestedReset = false;

void handleSerialCommands()
{
    if (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toLowerCase();

        if (command == "reset")
        {
            LOG_INFO_BLE("Reset flag set - will restart after current operations complete.");
            gRequestedReset = true;
        }
    }
}

void setup()
{
    initLogger();

    gServoService = new ServoService();

    const auto processingCallback = [](const RawMessage& command)
    {
        LOG_INFO_BLE("Processing incoming BLE message...");
        if (!gServoService)
        {
            LOG_ERROR_BLE("Servo service not initialized");
            return BluetoothService::BluetoothMessageResult::PROCESSING_FAILED;
        }
        ServoRequest servoCommand;
        if (!gServoService->parseServoRequest(command, servoCommand))
        {
            return BluetoothService::BluetoothMessageResult::VALIDATION_FAILED;
        }
        const auto result =
            gServoService->moveServos(servoCommand.s1_offset, servoCommand.s2_offset);
        if (result != ServoService::ServoActionResult::OK)
        {
            return BluetoothService::BluetoothMessageResult::PROCESSING_FAILED;
        }
        return BluetoothService::BluetoothMessageResult::OK;
    };
    gBleService = new BluetoothService(std::move(processingCallback));

    // Test servo movement on startup
    const auto result = gServoService->moveServos(90, 90);
    while (gServoService && (result == ServoService::ServoActionResult::OK))
    {
        gServoService->updateServos();
        if (gServoService->getVerticalServoState() == ServoState::IDLE &&
            gServoService->getHorizontalServoState() == ServoState::IDLE)
        {
            break;
        }
        delay(1);
    }
    delay(2000);
    const auto result2 = gServoService->moveServos(-180, -180);
    while (gServoService && (result2 == ServoService::ServoActionResult::OK))
    {
        gServoService->updateServos();
        if (gServoService->getVerticalServoState() == ServoState::IDLE &&
            gServoService->getHorizontalServoState() == ServoState::IDLE)
        {
            break;
        }
        delay(1);
    }
    delay(2000);
}

void loop()
{
    handleSerialCommands();
    if (gRequestedReset)
    {
        LOG_WARN_BLE("Resetting device as per serial command.");
        delay(100);
        ESP.restart();
    }

    while (gBleService && gBleService->isConnected())
    {
        gBleService->runStep();
        if (gServoService)
        {
            gServoService->updateServos();
        }
    }
    LOG_PERIODIC_BLE("Device disconnected. Atempting to recconect after 3 seconds.");
    sleep(3);
}