#include "MotorControl.h"

MotorControl Motors;

void MotorControl::begin() {
    // Setup Motor Pins
    pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT); pinMode(IN4_PIN, OUTPUT);
    
    // Setup Sensor Pins
    pinMode(FRONT_TRIG_PIN, OUTPUT); pinMode(FRONT_ECHO_PIN, INPUT);
    pinMode(BACK_TRIG_PIN, OUTPUT); pinMode(BACK_ECHO_PIN, INPUT);

    // --- MODERN LEDC SETUP (ESP32 Core v3.0+) ---
    // ledcAttach(pin, freq, resolution) automatically handles channels
    ledcAttach(ENA_PIN, PWM_FREQ, PWM_RES);
    ledcAttach(ENB_PIN, PWM_FREQ, PWM_RES);
    
    stop();
}

void MotorControl::setSpeed(int speed) {
    currentSpeed = constrain(speed, 0, 255);
    // ledcWrite now works directly with the pin
    ledcWrite(ENA_PIN, currentSpeed);
    ledcWrite(ENB_PIN, currentSpeed);
}

void MotorControl::stop() {
    digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, LOW);
    ledcWrite(ENA_PIN, 0);
    ledcWrite(ENB_PIN, 0);
    currentState = IDLE;
    Display.showStatus("Stopped");
}

void MotorControl::moveForward(int speed) {
    currentState = MOVING_FWD;
    digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW); // Right Fwd
    digitalWrite(IN3_PIN, LOW);  digitalWrite(IN4_PIN, HIGH); // Left Fwd
    setSpeed(speed);
    Display.showStatus("Fwd Spd:" + String(speed));
}

void MotorControl::moveBackward(int speed) {
    currentState = MOVING_BWD;
    digitalWrite(IN1_PIN, LOW);  digitalWrite(IN2_PIN, HIGH); // Right Bwd
    digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW);  // Left Bwd
    setSpeed(speed);
    Display.showStatus("Bwd Spd:" + String(speed));
}

void MotorControl::turnLeft(int speed) {
    currentState = TURNING_LEFT;
    digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW); // Right Fwd
    digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW); // Left Bwd
    setSpeed(speed);
    Display.showStatus("Turning Left");
}

void MotorControl::turnRight(int speed) {
    currentState = TURNING_RIGHT;
    digitalWrite(IN1_PIN, LOW);  digitalWrite(IN2_PIN, HIGH); // Right Bwd
    digitalWrite(IN3_PIN, LOW);  digitalWrite(IN4_PIN, HIGH); // Left Fwd
    setSpeed(speed);
    Display.showStatus("Turning Right");
}

RobotState MotorControl::getState() {
    return currentState;
}

float MotorControl::readUltrasonic(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW); delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    unsigned long duration = pulseIn(echoPin, HIGH, 30000);
    return (duration == 0) ? 999.0 : (duration * 0.0343 / 2.0);
}

float MotorControl::getFrontDistance() { return readUltrasonic(FRONT_TRIG_PIN, FRONT_ECHO_PIN); }
float MotorControl::getBackDistance() { return readUltrasonic(BACK_TRIG_PIN, BACK_ECHO_PIN); }