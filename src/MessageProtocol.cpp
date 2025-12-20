#include "MessageProtocol.hpp"

bool decodeServoRequest(const String& jsonString, ServoRequest& outCommand)
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

bool encodeAckResponse(const AckResponse& ack, String& outJsonString)
{
    JsonDocument doc;
    doc["msg_id"] = ack.msg_id;

    serializeJson(doc, outJsonString);
    return true;
}

bool encodeTaskFinishedResponse(const TaskFinishedResponse& response, String& outJsonString)
{
    JsonDocument doc;
    doc["msg_id"] = response.msg_id;

    serializeJson(doc, outJsonString);
    return true;
}
