#ifndef GROQCLIENT_H
#define GROQCLIENT_H

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "Config.h"
#include "env.h"

struct TranscriptionResult {
  String text;
  String error;
};

class GroqClient {
public:
    TranscriptionResult transcribe(const uint8_t* audioData, size_t audioSize);
    String getLlamaCommand(String text);
    void executeCommand(String jsonCommand);
};

extern GroqClient AI;

#endif