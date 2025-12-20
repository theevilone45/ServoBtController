#include "Config.hpp"
#include "Logger.hpp"
#include "MessageProtocol.hpp"
#include "ServoService.hpp"
#include "BluetoothService.hpp"

BluetoothService* gBleService;
ServoService* gServoService;
bool gRequestedReset = false;

void fatalError(const String& errorMessage)
{
    LOG_ERROR_BLE("Fatal error, reset by button: " + errorMessage);
    pinMode(2, OUTPUT); // ESP32 built-in LED is typically on pin 2
    while(true) {
        digitalWrite(2, HIGH);
        delay(200);
        digitalWrite(2, LOW);
        delay(200);
    }
}

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

        const auto result = gServoService->moveServos(servoCommand.v_offset, servoCommand.h_offset);
        if (result != ServoService::ServoActionResult::IN_PROGRESS)
        {
            return BluetoothService::BluetoothMessageResult::PROCESSING_FAILED;
        }
        
        gServoService->setCurrentMsgId(servoCommand.msg_id);
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
