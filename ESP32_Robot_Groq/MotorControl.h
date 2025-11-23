#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include "Config.h"
#include "LCDHandler.h"

// Enum arah gerak
enum RobotState { IDLE, MOVING_FWD, MOVING_BWD, TURNING_LEFT, TURNING_RIGHT };

class MotorControl {
public:
    void begin();
    void stop();
    void moveForward(int speed);
    void moveBackward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void setSpeed(int speed);
    RobotState getState();
    
    // Fungsi sensor ultrasonik sederhana untuk safety
    float getFrontDistance();
    float getBackDistance();

private:
    RobotState currentState = IDLE;
    int currentSpeed = 0;
    
    float readUltrasonic(int trigPin, int echoPin);
};

extern MotorControl Motors;

#endif