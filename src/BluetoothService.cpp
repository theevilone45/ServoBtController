#include "BluetoothService.hpp"

BluetoothService::BluetoothService(ProcessingCallbackType&& processingCallback)
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

bool BluetoothService::isCriticalError(BluetoothMessageResult result) const
{
    return static_cast<int32_t>(result) >= CRITICAL_BLUETOOTH_ERROR;
}

void BluetoothService::runStep()
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

bool BluetoothService::isConnected() const
{
    return getCentralDevice().connected();
}

String BluetoothService::receive()
{
    if (mMessageService.written())
    {
        String incoming = mMessageService.value();
        LOG_INFO_BLE("Received: " + incoming);
        return incoming;
    }
    return String();
}

bool BluetoothService::send(const String& message)
{
    LOG_INFO_BLE("Sending: " + message);
    return mMessageService.writeValue(message);
}

bool BluetoothService::sendAckResponse(uint16_t msg_id)
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

bool BluetoothService::sendTaskFinishedResponse(uint16_t msg_id)
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

BLEDevice BluetoothService::getCentralDevice() const
{
    return BLE.central();
}
