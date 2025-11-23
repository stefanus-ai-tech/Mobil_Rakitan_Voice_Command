#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ======================= PIN DEFINITIONS (PERMANENT) =======================
const int SDA_PIN = 21; const int SCL_PIN = 22;
const int ENA_PIN = 32;  // Left Motor PWM
const int ENB_PIN = 33;  // Right Motor PWM
const int IN1_PIN = 12;  // Right Motor Pin 1
const int IN2_PIN = 13;  // Right Motor Pin 2
const int IN3_PIN = 14;  // Left Motor Pin 1
const int IN4_PIN = 27;  // Left Motor Pin 2
const int FRONT_TRIG_PIN = 5; const int FRONT_ECHO_PIN = 18;
const int BACK_TRIG_PIN = 19; const int BACK_ECHO_PIN = 23;

// ======================= CONSTANTS =======================
const int PWM_FREQ = 5000;
const int PWM_RES = 8; // 8-bit resolution (0-255)
const float OBSTACLE_THRESHOLD_CM = 30.0;
const unsigned long TURN_TIMEOUT = 5000;
const int TURN_PWM = 200;

// API URLs
#define whisper_api_url "https://api.groq.com/openai/v1/audio/transcriptions"
#define llama_api_url "https://api.groq.com/openai/v1/chat/completions"

#endif