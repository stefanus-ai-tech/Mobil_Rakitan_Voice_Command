#include "GroqClient.h"
#include "MotorControl.h"
#include "LCDHandler.h"

GroqClient AI;

TranscriptionResult GroqClient::transcribe(const uint8_t* audioData, size_t audioSize) {
    TranscriptionResult result;
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;

    // --- GROQ WHISPER LOGIC (UNCHANGED) ---
    if (http.begin(client, whisper_api_url)) {
        http.addHeader("Authorization", "Bearer " + String(GROQ_API_KEY));
        String boundary = "----WebKitFormBoundaryESP32";
        http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);

        String head = "--" + boundary + "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"cmd.webm\"\r\nContent-Type: audio/webm\r\n\r\n";
        String tail = "\r\n--" + boundary + "\r\nContent-Disposition: form-data; name=\"model\"\r\n\r\nwhisper-large-v3\r\n--" + boundary + "--\r\n";

        size_t totalLen = head.length() + audioSize + tail.length();
        uint8_t* buf = (uint8_t*)malloc(totalLen);
        if (!buf) { result.error = "OOM"; return result; }

        memcpy(buf, head.c_str(), head.length());
        memcpy(buf + head.length(), audioData, audioSize);
        memcpy(buf + head.length() + audioSize, tail.c_str(), tail.length());

        int code = http.POST(buf, totalLen);
        free(buf);

        if (code == 200) {
            JsonDocument doc;
            deserializeJson(doc, http.getString());
            result.text = doc["text"].as<String>();
        } else {
            result.error = "HTTP " + String(code);
        }
        http.end();
    } else { result.error = "Conn Failed"; }
    return result;
}

String GroqClient::getLlamaCommand(String text) {
    String command = "";
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;

    if (http.begin(client, llama_api_url)) {
        http.addHeader("Authorization", "Bearer " + String(GROQ_API_KEY));
        http.addHeader("Content-Type", "application/json");

        JsonDocument doc;
        doc["model"] = "llama3-8b-8192";
        JsonArray msgs = doc.createNestedArray("messages");
        msgs.add<JsonObject>()["role"] = "system";
        msgs[0]["content"] = "You are a robot. Respond ONLY JSON: {\"command\":\"forward|backward|turn_left|turn_right|stop\", \"angle\":45}. Default angle 45.";
        msgs.add<JsonObject>()["role"] = "user";
        msgs[1]["content"] = text;
        doc["response_format"]["type"] = "json_object";

        String payload; serializeJson(doc, payload);
        if (http.POST(payload) == 200) {
            JsonDocument resp; deserializeJson(resp, http.getString());
            command = resp["choices"][0]["message"]["content"].as<String>();
        }
        http.end();
    }
    return command;
}

void GroqClient::executeCommand(String jsonCommand) {
    JsonDocument doc;
    if (deserializeJson(doc, jsonCommand)) { Display.showMessage("JSON Error"); return; }
    
    String cmd = doc["command"];
    int angle = doc["angle"] | 45; // Bisa digunakan untuk durasi delay turn
    
    Display.showMessage("Exec: " + cmd, "Angle: " + String(angle));

    if (cmd == "forward") Motors.moveForward(255);
    else if (cmd == "backward") Motors.moveBackward(255);
    else if (cmd == "stop") Motors.stop();
    else if (cmd == "turn_left") {
        Motors.turnLeft(200);
        delay(angle * 15); // Estimasi kasar 15ms per derajat
        Motors.stop();
    } else if (cmd == "turn_right") {
        Motors.turnRight(200);
        delay(angle * 15);
        Motors.stop();
    }
}