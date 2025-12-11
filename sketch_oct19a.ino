#include <ArduinoBLE.h>
#include <ArduinoJson.h>
#include <functional>
#include <memory>

namespace
{

constexpr const char* cTextServiceUuid = "12345678-1234-1234-1234-1234567890ab";
constexpr const char* cMessageServiceUuid = "abcd1234-1234-1234-1234-1234567890ab";
constexpr const char* cLocalName = "ArduinoBL";

#ifndef MIN_LOG_LEVEL
#define MIN_LOG_LEVEL 1 // 0 - DEBUG, 1 - INFO, 2 - WARN, 3 - ERROR
#endif

enum class ModuleId
{
    BLE = 0,
    SERVO,
};

enum class Severity
{
    DEBUG = 0,
    INFO = 1,
    WARN = 2,
    ERROR = 3
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

using BleDebugLogger = Logger<ModuleId::BLE, Severity::DEBUG>;
using BleInfoLogger = Logger<ModuleId::BLE, Severity::INFO>;
using BleWarnLogger = Logger<ModuleId::BLE, Severity::WARN>;
using BleErrorLogger = Logger<ModuleId::BLE, Severity::ERROR>;

using ServoDebugLogger = Logger<ModuleId::SERVO, Severity::DEBUG>;
using ServoInfoLogger = Logger<ModuleId::SERVO, Severity::INFO>;
using ServoWarnLogger = Logger<ModuleId::SERVO, Severity::WARN>;
using ServoErrorLogger = Logger<ModuleId::SERVO, Severity::ERROR>;

} // namespace

#define LOG_DEBUG_BLE(msg) BleDebugLogger::log(msg)
#define LOG_INFO_BLE(msg) BleInfoLogger::log(msg)
#define LOG_WARN_BLE(msg) BleWarnLogger::log(msg)
#define LOG_ERROR_BLE(msg) BleErrorLogger::log(msg)

#define LOG_DEBUG_SERVO(msg) ServoDebugLogger::log(msg)
#define LOG_INFO_SERVO(msg) ServoInfoLogger::log(msg)
#define LOG_WARN_SERVO(msg) ServoWarnLogger::log(msg)
#define LOG_ERROR_SERVO(msg) ServoErrorLogger::log(msg)

struct ServoCommand
{
    uint16_t msg_id;
    int16_t s1_offset; // up / down
    int16_t s2_offset; // left / right
};

struct AckMessage
{
    uint16_t msg_id;
    bool success;
    String error_message;
};

[[nodiscard]] bool decodeServoCommand(const String& jsonString, ServoCommand& outCommand)
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

[[nodiscard]] bool decodeAckMessage(const String& jsonString, AckMessage& outAck)
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

[[nodiscard]] bool encodeServoCommand(const ServoCommand& command, String& outJsonString)
{
    JsonDocument doc;
    doc["msg_id"] = command.msg_id;
    doc["s1_offset"] = command.s1_offset;
    doc["s2_offset"] = command.s2_offset;

    serializeJson(doc, outJsonString);
    return true;
}

[[nodiscard]] bool encodeAckMessage(const AckMessage& ack, String& outJsonString)
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
    enum class ServoActionResult
    {
        OK = 0,
        OUT_OF_RANGE,
        DECODE_FAILURE,
        COMMUNICATION_FAILURE,
        UNKNOWN_FAILURE
    };
    using ReceivingCallbackType = std::function<ServoActionResult(const ServoCommand&)>;

  public:
    BluetoothService(ReceivingCallbackType&& receivingCallback)
        : mReceivingCallback(std::move(receivingCallback)), mTextService(cTextServiceUuid),
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
        mIsAdvertising = true;
    }

    void runStep()
    {
        LOG_DEBUG_BLE("Running BLE step");
        BLE.poll();
        String incoming = receive();
        if (incoming.length() > 0)
        {
            ServoCommand command;
            if (!decodeServoCommand(incoming, command))
            {
                LOG_ERROR_BLE("Failed to decode incoming servo command");
                return;
            }
            const auto result = mReceivingCallback(command);
            // Send acknowledgment
            AckMessage ack;
            ack.msg_id = command.msg_id;
            ack.success = (result == ServoActionResult::OK);
            if (!ack.success)
            {
                ack.error_message =
                    "Servo action failed with code: " + String(static_cast<int>(result));
            }
            LOG_DEBUG_BLE("Sending acknowledgment for msg_id: " + String(ack.msg_id));
            String ackJson;
            if (!encodeAckMessage(ack, ackJson))
            {
                LOG_ERROR_BLE("Failed to encode acknowledgment message");
                return;
            }
            send(ackJson);
        }
    }

    [[nodiscard]] bool isAdvertising() const
    {
        return mIsAdvertising;
    }

    [[nodiscard]] BLEDevice getCentralDevice() const
    {
        return BLE.central();
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

    void send(const String& message)
    {
        LOG_INFO_BLE("Sending: " + message);
        mMessageService.writeValue(message);
    }

    [[nodiscard]] BLEStringCharacteristic& getMessageService()
    {
        return mMessageService;
    }

  private:
    const ReceivingCallbackType mReceivingCallback;
    BLEService mTextService;
    BLEStringCharacteristic mMessageService;
    bool mIsAdvertising = false;
};

std::shared_ptr<BluetoothService> bleService;

void setup()
{
    initLogger();

    LOG_INFO_BLE("Starting BLE...");

    const auto recevingCallback = [](const ServoCommand& command)
    {
        LOG_INFO_BLE("Received Servo Command - msg_id: " + String(command.msg_id) +
                     ", s1_offset: " + String(command.s1_offset) +
                     ", s2_offset: " + String(command.s2_offset));
        return BluetoothService::ServoActionResult::OK;
    };

    bleService = std::make_shared<BluetoothService>(std::move(recevingCallback));

    LOG_INFO_BLE("Waiting for central...");
}

void loop()
{
    while (bleService->isConnected())
    {
        bleService->runStep();
    }
    LOG_WARN_BLE("Device disconnected. Atempting to recconect after 3 seconds.");
    sleep(3);
}