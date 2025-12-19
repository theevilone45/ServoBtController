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

constexpr const int cVerticalServoPin = 5;
constexpr const int cHorizontalServoPin = 4;
constexpr const int cVerticalServoDelayMs = 1;
constexpr const int cHorizontalServoDelayMs = 1;

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
    int16_t h_offset; // left / right
    int16_t v_offset; // up / down
};

struct AckResponse
{
    uint16_t msg_id;
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
    outCommand.h_offset = doc["h_offset"];
    outCommand.v_offset = doc["v_offset"];

    return true;
}

[[nodiscard]] bool encodeAckResponse(const AckResponse& ack, String& outJsonString)
{
    JsonDocument doc;
    doc["msg_id"] = ack.msg_id;

    serializeJson(doc, outJsonString);
    return true;
}

[[nodiscard]] bool encodeTaskFinishedResponse(const TaskFinishedResponse& response,
                                               String& outJsonString)
{
    JsonDocument doc;
    doc["msg_id"] = response.msg_id;

    serializeJson(doc, outJsonString);
    return true;
}

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

    bool isCriticalError(BluetoothMessageResult result) const
    {
        return static_cast<int32_t>(result) >= CRITICAL_BLUETOOTH_ERROR;
    }

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
            if(processingResult == BluetoothMessageResult::OK)
            {
                LOG_INFO_BLE("Message processed successfully");
            }
            else
            {
                LOG_WARN_BLE("Message processing resulted in: " +
                              String(static_cast<int>(processingResult)));
            }
            if (isCriticalError(processingResult))
            {
                LOG_ERROR_BLE("Error while processing: " + static_cast<int>(processingResult));
                fatalError("Critical BLE processing error");
            }
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

    [[nodiscard]] bool sendAckResponse(uint16_t msg_id)
    {
        AckResponse ackResponse;
        ackResponse.msg_id = msg_id;
        String ackJson;
        if (!encodeAckResponse(ackResponse, ackJson))
        {
            LOG_ERROR_BLE("Failed to encode ACK response");
            return false;
        }
        if (!send(ackJson))
        {
            LOG_ERROR_BLE("Failed to send ACK response");
            return false;
        }
        return true;
    }

    [[nodiscard]] bool sendTaskFinishedResponse(uint16_t msg_id)
    {
        TaskFinishedResponse taskFinishedResponse;
        taskFinishedResponse.msg_id = msg_id;
        String responseJson;
        if (!encodeTaskFinishedResponse(taskFinishedResponse, responseJson))
        {
            LOG_ERROR_BLE("Failed to encode Task Finished response");
            return false;
        }
        if (!send(responseJson))
        {
            LOG_ERROR_BLE("Failed to send Task Finished response");
            return false;
        }
        return true;
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

  public:
    ServoService()
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

    [[nodiscard]] bool initServos()
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

    void updateServos()
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

    [[nodiscard]] ServoActionResult moveServos(int16_t verticalOffset, int16_t horizontalOffset)
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

    [[nodiscard]] bool parseServoRequest(const RawMessage& rawMessage, ServoRequest& command)
    {
        if (!decodeServoRequest(rawMessage, command))
        {
            LOG_ERROR_SERVO("Failed to decode incoming servo command");
            return false;
        }
        // if (command.h_offset > 180 || command.v_offset > 180 || command.h_offset < -180 ||
        //     command.v_offset < -180)
        // {
        //     LOG_WARN_SERVO("Received servo command with out of range offsets. Ignoring.");
        //     return false;
        // }
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

    [[nodiscard]] bool areServosIdle() const
    {
        return (mVerticalServoDevice.state == ServoState::IDLE) &&
               (mHorizontalServoDevice.state == ServoState::IDLE);
    }

    [[nodiscard]] uint16_t getCurrentMsgId() const
    {
        return mCurrentMsgId;
    }

    void setCurrentMsgId(uint16_t msgId)
    {
        mCurrentMsgId = msgId;
    }

  private:
    [[nodiscard]] int getHorizontalOffset() const
    {
        return mHorizontalServoDevice.targetPosition - mHorizontalServoDevice.currentPosition;
    }

    [[nodiscard]] int getVerticalOffset() const
    {
        return mVerticalServoDevice.targetPosition - mVerticalServoDevice.currentPosition;
    }

    void updateHorizontal(unsigned long currentMillis)
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

    void updateVertical(unsigned long currentMillis)
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

    ServoDevice mHorizontalServoDevice;
    ServoDevice mVerticalServoDevice;
    unsigned long mLastHorizontalUpdateTime = 0;
    unsigned long mLastVerticalUpdateTime = 0;
    uint16_t mCurrentMsgId = 0;
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

void runTestRoutine()
{
    const auto result = gServoService->moveServos(45, 90);
    while (gServoService && (result == ServoService::ServoActionResult::IN_PROGRESS))
    {
        gServoService->updateServos();
        if (gServoService->getVerticalServoState() == ServoState::IDLE &&
            gServoService->getHorizontalServoState() == ServoState::IDLE)
        {
            break;
        }
    }
    delay(500);
    const auto result2 = gServoService->moveServos(-90, -180);
    while (gServoService && (result2 == ServoService::ServoActionResult::IN_PROGRESS))
    {
        gServoService->updateServos();
        if (gServoService->getVerticalServoState() == ServoState::IDLE &&
            gServoService->getHorizontalServoState() == ServoState::IDLE)
        {
            break;
        }
    }
    delay(500);
    const auto result3 = gServoService->moveServos(45, 90);
    while (gServoService && (result3 == ServoService::ServoActionResult::IN_PROGRESS))
    {
        gServoService->updateServos();
        if (gServoService->getVerticalServoState() == ServoState::IDLE &&
            gServoService->getHorizontalServoState() == ServoState::IDLE)
        {
            break;
        }
    }
    delay(500);
}

void fatalError(const String& errorMessage)
{
    LOG_ERROR_BLE("Fatal error, reset by button: " + errorMessage);
    pinMode(LED_BUILTIN, OUTPUT);
    while(true) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
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
            return BluetoothService::BluetoothMessageResult::SERVO_UNINITIALIZED;
        }

        if(!gServoService->areServosIdle())
        {
            LOG_WARN_BLE("Servos are busy. Cannot process new command.");
            return BluetoothService::BluetoothMessageResult::ROBOT_BUSY;
        }
        else if(gServoService->getCurrentMsgId() != 0)
        {
            LOG_INFO_BLE("Previous task finished. Sending Task Finished response.");
            if(!gBleService->sendTaskFinishedResponse(gServoService->getCurrentMsgId()))
            {
                LOG_ERROR_BLE("Failed to send Task Finished response");
                return BluetoothService::BluetoothMessageResult::SENDING_MESSAGE_FAILED;
            }
            gServoService->setCurrentMsgId(0);
        }

        ServoRequest servoCommand;
        if (!gServoService->parseServoRequest(command, servoCommand))
        {
            return BluetoothService::BluetoothMessageResult::VALIDATION_FAILED;
        }

        if(!gBleService->sendAckResponse(servoCommand.msg_id))
        {
            LOG_ERROR_BLE("Failed to send ACK response");
            return BluetoothService::BluetoothMessageResult::SENDING_MESSAGE_FAILED;
        }

        const auto result = gServoService->moveServos(servoCommand.h_offset, servoCommand.v_offset);
        if (result != ServoService::ServoActionResult::IN_PROGRESS)
        {
            return BluetoothService::BluetoothMessageResult::PROCESSING_FAILED;
        }
        
        return BluetoothService::BluetoothMessageResult::OK;
    };
    gBleService = new BluetoothService(std::move(processingCallback));

    LOG_INFO_BLE("Setup complete. Running test routine...");
    runTestRoutine();
    LOG_INFO_BLE("Test routine complete. Entering main loop.");
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
        delay(1);
    }
    LOG_PERIODIC_BLE("Device disconnected. Atempting to recconect after 3 seconds.");
    sleep(3);
}