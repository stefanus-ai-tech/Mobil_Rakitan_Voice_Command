#include "NetworkManager.h"
#include "MotorControl.h"
#include "GroqClient.h"
#include "LCDHandler.h"

MyNetworkManager myNetwork;
AsyncWebServer server(80);

void MyNetworkManager::begin() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    Display.showMessage("Connecting WiFi", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    
    Display.showMessage("IP Address:", WiFi.localIP().toString());
    Serial.println(WiFi.localIP());

    // --- WEB ROUTES ---
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send(200, "text/html", index_html); });
    
    server.on("/forward", HTTP_GET, [](AsyncWebServerRequest *r){ 
        Motors.moveForward(r->hasParam("speed") ? r->getParam("speed")->value().toInt() : 200);
        r->send(200, "text/plain", "OK"); 
    });
    server.on("/backward", HTTP_GET, [](AsyncWebServerRequest *r){ 
        Motors.moveBackward(r->hasParam("speed") ? r->getParam("speed")->value().toInt() : 200);
        r->send(200, "text/plain", "OK"); 
    });
    server.on("/turn_left", HTTP_GET, [](AsyncWebServerRequest *r){ Motors.turnLeft(200); r->send(200, "text/plain", "OK"); });
    server.on("/turn_right", HTTP_GET, [](AsyncWebServerRequest *r){ Motors.turnRight(200); r->send(200, "text/plain", "OK"); });
    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *r){ Motors.stop(); r->send(200, "text/plain", "OK"); });

    // --- VOICE UPLOAD ---
    server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *r){
        r->send(200, "application/json", "{\"message\":\"Processing...\"}");
    }, handleUpload);

    server.begin();
}

// Handler Audio Upload yang membersihkan logic main
void MyNetworkManager::handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    static uint8_t* audioBuffer = NULL;
    static size_t bufferSize = 0;

    if (!index) {
        if(audioBuffer) { free(audioBuffer); audioBuffer = NULL; }
        audioBuffer = (uint8_t*)malloc(len);
        bufferSize = len;
        if(audioBuffer) memcpy(audioBuffer, data, len);
    } else {
        uint8_t* newBuf = (uint8_t*)realloc(audioBuffer, bufferSize + len);
        if(newBuf) { audioBuffer = newBuf; memcpy(audioBuffer + bufferSize, data, len); bufferSize += len; }
    }

    if (final && audioBuffer) {
        Display.showMessage("Transcribing...");
        TranscriptionResult res = AI.transcribe(audioBuffer, bufferSize);
        free(audioBuffer); audioBuffer = NULL;

        if (res.error.length() == 0) {
            Display.showMessage("Thinking...");
            String cmdJson = AI.getLlamaCommand(res.text);
            AI.executeCommand(cmdJson);
        } else {
            Display.showMessage("Err: " + res.error);
        }
    }
}