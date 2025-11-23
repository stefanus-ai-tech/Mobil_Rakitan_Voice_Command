#ifndef NETWORKMANAGER_H
#define NETWORKMANAGER_H

#include <ESPAsyncWebServer.h>
#include "Config.h"
#include "env.h"
#include "WebInterface.h"

class MyNetworkManager {
public:
    void begin();
    void handleClient(); // Loop handler jika perlu
private:
    static void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
};

extern MyNetworkManager myNetwork;

#endif