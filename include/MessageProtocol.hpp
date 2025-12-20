#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

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

[[nodiscard]] bool decodeServoRequest(const String& jsonString, ServoRequest& outCommand);
[[nodiscard]] bool encodeAckResponse(const AckResponse& ack, String& outJsonString);
[[nodiscard]] bool encodeTaskFinishedResponse(const TaskFinishedResponse& response, String& outJsonString);
