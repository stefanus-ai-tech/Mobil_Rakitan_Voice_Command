#include <Arduino.h>
#include "Config.h"
#include "LCDHandler.h"
#include "MotorControl.h"
#include "NetworkManager.h"

// Main Setup
void setup() {
    Serial.begin(115200);
    
    // 1. Init LCD
    Display.begin();
    
    // 2. Init Motors (LEDC setup happens here)
    Motors.begin();
    
    // 3. Init Network & Server
    Network.begin();
    
    Display.showMessage("System Ready", "Waiting Cmd...");
}

// Main Loop - Hanya untuk safety monitoring
void loop() {
    // Check Obstacle
    if (Motors.getState() == MOVING_FWD && Motors.getFrontDistance() < OBSTACLE_THRESHOLD_CM) {
        Motors.stop();
        Display.showMessage("OBSTACLE DETECTED", "FRONT STOP");
        delay(1000);
    }
    else if (Motors.getState() == MOVING_BWD && Motors.getBackDistance() < OBSTACLE_THRESHOLD_CM) {
        Motors.stop();
        Display.showMessage("OBSTACLE DETECTED", "BACK STOP");
        delay(1000);
    }
    
    delay(50); // Small delay for stability
}