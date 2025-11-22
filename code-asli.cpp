// ===================================================================================
//
//      ESP32 Web Server Robot with Voice Control via Groq API (v3 - Final)
//      Corrects all compilation errors for ESP32 Arduino Core v3.x+
//
// ===================================================================================



// --- Library Includes ---
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <QMC5883LCompass.h>
#include "driver/ledc.h"      // Modern LEDC (PWM) driver
#include "driver/pulse_cnt.h" // Modern Pulse Counter driver

// --- WiFi Credentials ---
const char* ssid = "masukan nama SSID wifi";       // Your WiFi Network Name
const char* password = "masukan password wifi"; // Your WiFi Password

// --- Groq API Configuration ---
// ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
//  >>>>> IMPORTANT! YOU MUST REPLACE THIS WITH YOUR REAL GROQ API KEY! <<<<<
// ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
const char* groq_api_key = "gsk_masukanRealAPIKeydariGroq.com";

const char* whisper_api_url = "https://api.groq.com/openai/v1/audio/transcriptions";
const char* llama_api_url = "https://api.groq.com/openai/v1/chat/completions";


// ===================================================================================
//                             RESULT STRUCTURE
// ===================================================================================
// This struct will be used to return detailed results from the transcription function.
struct TranscriptionResult {
  String text;  // Will contain the transcribed text on success.
  String error; // Will contain a specific error message on failure.
};


// ======================= RPM SENSOR DEFINITIONS =======================
#define LEFT_RPM_SENSOR_PIN  26
#define RIGHT_RPM_SENSOR_PIN 25
#define PULSES_PER_REVOLUTION 20
#define RPM_READ_INTERVAL 100
pcnt_unit_handle_t left_pcnt_unit = NULL;
pcnt_unit_handle_t right_pcnt_unit = NULL;
volatile int leftRPM = 0;
volatile int rightRPM = 0;
unsigned long lastRpmReadTime = 0;

// ======================= COMPASS & TURNING CONFIG =======================
unsigned long lastCompassRead = 0;
const unsigned long COMPASS_READ_INTERVAL = 50;
const float HEADING_TOLERANCE = 2.0;
const float SLOW_DOWN_ANGLE = 15.0;
const int MIN_TURN_PWM = 150;
const int DEFAULT_TURN_PWM = 300;

// ======================= PIN DEFINITIONS =======================
const int SDA_PIN = 21; const int SCL_PIN = 22;
const int ENA_PIN = 32;  // Left Motor PWM
const int ENB_PIN = 33;  // Right Motor PWM
const int IN1_PIN = 12;  // Right Motor Pin 1
const int IN2_PIN = 13;  // Right Motor Pin 2
const int IN3_PIN = 14;  // Left Motor Pin 1
const int IN4_PIN = 27;  // Left Motor Pin 2
const int FRONT_TRIG_PIN = 5; const int FRONT_ECHO_PIN = 18;
const int BACK_TRIG_PIN = 19; const int BACK_ECHO_PIN = 23;

// ======================= COMPONENT OBJECTS =======================
QMC5883LCompass compass;
LiquidCrystal_I2C lcd(0x27, 16, 2);
AsyncWebServer server(80);

// ======================= MOTOR & MOVEMENT CONFIG =======================
#define MOTOR_STOP      0
#define MOTOR_FORWARD   1
#define MOTOR_BACKWARD  2
int motorCalibOffset = 0;
const int TURN_PWM = 255; // Use maximum power for turns
const int MS_PER_DEGREE_OF_TURN = 12; // IMPORTANT: Calibrate this value for your robot!

// ======================= OBSTACLE DETECTION CONFIG =======================
const float OBSTACLE_THRESHOLD_CM = 30.0;
const int SENSOR_READ_INTERVAL = 50;

// ======================= GLOBAL STATE & PWM =======================
float logicalFrontDistance = 999;
float logicalBackDistance = 999;
int currentSpeed = 0;
unsigned long lastReconnectAttempt = 0;
const int PWM_FREQ = 5000;
// Note: We use LEDC_LOW_SPEED_MODE which requires a specific timer and channel setup
ledc_timer_t      pwm_timer      = LEDC_TIMER_0;
ledc_channel_t    left_pwm_ch    = LEDC_CHANNEL_0;
ledc_channel_t    right_pwm_ch   = LEDC_CHANNEL_1;
ledc_mode_t       pwm_mode       = LEDC_LOW_SPEED_MODE;

// ======================= STATE MACHINE =======================
enum RobotState { IDLE, MOVING_FWD, MOVING_BWD, TURNING };
volatile RobotState currentState = IDLE;

// ======================= NON-BLOCKING VARS =======================
float turnTargetHeading = 0, turnStartHeading = 0;
int turnSpeed = 0;
unsigned long turnStartTime = 0;
const unsigned long TURN_TIMEOUT = 10000;


// ======================= HTML, CSS & JAVASCRIPT FOR WEB UI =======================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP32 Robot Voice Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; text-align: center; margin-top: 20px; background-color: #2c3e50; color: #ecf0f1; }
    h1 { color: #3498db; }
    .btn-container { display: grid; grid-template-columns: 1fr 1fr 1fr; grid-template-rows: auto; gap: 10px; max-width: 320px; margin: 20px auto; }
    .btn { background-color: #3498db; border: none; color: white; padding: 20px; text-align: center; text-decoration: none; font-size: 18px; cursor: pointer; border-radius: 10px; user-select: none; transition: all 0.2s; }
    .btn:hover { background-color: #2980b9; }
    .btn.active { background-color: #27ae60; box-shadow: 0 0 10px rgba(39, 174, 96, 0.5); }
    .forward { grid-column: 2; grid-row: 1; }
    .turn-left { grid-column: 1; grid-row: 2; }
    .stop { grid-column: 2; grid-row: 2; background-color: #e74c3c; font-weight: bold; }
    .stop:hover { background-color: #c0392b; }
    .turn-right { grid-column: 3; grid-row: 2; }
    .backward { grid-column: 2; grid-row: 3; }
    .voice-btn { background-color: #9b59b6; grid-column: 1 / -1; grid-row: 4; margin-top: 10px; }
    .voice-btn:hover { background-color: #8e44ad; }
    .voice-btn.recording { background-color: #e74c3c; animation: pulse 1s infinite; }
    @keyframes pulse { 0% { box-shadow: 0 0 0 0 rgba(231, 76, 60, 0.7); } 70% { box-shadow: 0 0 0 20px rgba(231, 76, 60, 0); } 100% { box-shadow: 0 0 0 0 rgba(231, 76, 60, 0); } }
    #status { margin-top: 20px; font-size: 1.2em; color: #f1c40f; min-height:1.5em; }
    .slider-container { margin: 20px auto; max-width: 300px; }
    .control-box { background-color: #34495e; padding: 15px; border-radius: 10px; margin-top: 20px; }
    label { font-size: 1.1em; }
    #speedValue, #turnAngleValue { font-weight: bold; color: #3498db; }
    .slider { width: 100%; }
  </style>
</head>
<body>
  <h1>ESP32 Robot Control</h1>
  <div class="btn-container">
    <button class="btn forward"    onclick="toggleCommand('forward')">Forward</button>
    <button class="btn turn-left"  onclick="sendCommand('turn_left')">Turn Left</button>
    <button class="btn stop"       onclick="sendCommand('stop')">Stop</button>
    <button class="btn turn-right" onclick="sendCommand('turn_right')">Turn Right</button>
    <button class="btn backward"   onclick="toggleCommand('backward')">Backward</button>
    <button id="voiceBtn" class="btn voice-btn" ontouchstart="handleVoiceStart(event)" ontouchend="handleVoiceEnd(event)" onmousedown="handleVoiceStart(event)" onmouseup="handleVoiceEnd(event)">Hold for Voice Command</button>
  </div>
  <p id="status">Click a direction or use voice.</p>
  <div class="control-box">
    <div class="slider-container">
      <label for="speedSlider">Speed: <span id="speedValue">200</span></label>
      <input type="range" min="150" max="255" value="200" class="slider" id="speedSlider">
    </div>
    <div class="slider-container">
      <label for="turnAngleSlider">Turn Angle: <span id="turnAngleValue">90</span>°</label>
      <input type="range" min="15" max="180" value="90" class="slider" id="turnAngleSlider">
    </div>
  </div>
  <script>
    let currentMoveCommand = 'stop';
    document.addEventListener('DOMContentLoaded', (event) => {
      const speedSlider = document.getElementById('speedSlider');
      const speedValueSpan = document.getElementById('speedValue');
      const turnSlider = document.getElementById('turnAngleSlider');
      const turnAngleValueSpan = document.getElementById('turnAngleValue');
      speedSlider.oninput = function() {
        speedValueSpan.innerHTML = this.value;
        if (currentMoveCommand !== 'stop') { sendSpeedUpdate(this.value); }
      }
      turnSlider.oninput = function() { turnAngleValueSpan.innerHTML = this.value; }
    });
    function toggleCommand(command) { if (currentMoveCommand === command) { sendCommand('stop'); } else { sendCommand(command); } }
    function sendCommand(command) {
      let url = '/' + command; let statusText = 'Sending: ' + command.replace('_', ' ');
      if (command === 'forward' || command === 'backward' || command === 'turn_left' || command === 'turn_right') { url += '?speed=' + document.getElementById('speedSlider').value; }
      if (command === 'turn_left' || command === 'turn_right') { url += '&angle=' + document.getElementById('turnAngleSlider').value; }
      document.getElementById('status').innerText = statusText;
      if (command === 'forward' || command === 'backward' || command === 'stop') { updateButtonStates(command); currentMoveCommand = command; } 
      else { updateButtonStates('stop'); currentMoveCommand = 'stop'; }
      fetch(url).catch(error => { document.getElementById('status').innerText = 'Connection Error!'; updateButtonStates('stop'); currentMoveCommand = 'stop'; });
    }
    function sendSpeedUpdate(speed) { fetch('/speed?value=' + speed).then(response => { if (response.ok && currentMoveCommand !== 'stop') { document.getElementById('status').innerText = 'Moving: ' + currentMoveCommand + ' at speed ' + speed; } }); }
    function updateButtonStates(activeCommand) {
      document.querySelectorAll('.btn.forward, .btn.backward').forEach(btn => btn.classList.remove('active'));
      if (activeCommand === 'forward' || activeCommand === 'backward') {
        const activeBtn = document.querySelector('.' + activeCommand);
        if (activeBtn) activeBtn.classList.add('active');
        document.getElementById('status').innerText = 'Moving: ' + activeCommand + ' at speed ' + document.getElementById('speedSlider').value;
      } else { document.getElementById('status').innerText = 'Robot stopped.'; }
    }
    let mediaRecorder; let audioChunks = []; const voiceBtn = document.getElementById('voiceBtn');
    async function handleVoiceStart(event) {
      event.preventDefault();
      try {
        const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
        mediaRecorder = new MediaRecorder(stream);
        mediaRecorder.start();
        audioChunks = [];
        voiceBtn.classList.add('recording'); voiceBtn.innerText = "Recording...";
        document.getElementById('status').innerText = "Listening...";
        mediaRecorder.addEventListener("dataavailable", e => { audioChunks.push(e.data); });
        mediaRecorder.addEventListener("stop", () => {
          const audioBlob = new Blob(audioChunks, { type: 'audio/wav' });
          sendAudioToServer(audioBlob);
          stream.getTracks().forEach(track => track.stop());
        });
      } catch (error) { 
        console.error("Mic error:", error); 
        document.getElementById('status').innerText = "Mic Error: Use https or enable insecure origins flag in browser."; 
      }
    }
    function handleVoiceEnd(event) { 
        event.preventDefault(); 
        if (mediaRecorder && mediaRecorder.state === "recording") { 
            mediaRecorder.stop(); 
            voiceBtn.classList.remove('recording'); 
            voiceBtn.innerText = "Hold for Voice Command"; 
        } 
    }
    function sendAudioToServer(audioBlob) {
      document.getElementById('status').innerText = "Uploading & Processing...";
      const formData = new FormData();
      formData.append("audio", audioBlob, "command.wav");
      fetch("/upload", { method: "POST", body: formData })
        .then(response => {
            if (!response.ok) { throw new Error('Network response: ' + response.statusText); }
            return response.json();
        })
        .then(data => { document.getElementById('status').innerText = data.message || "Processed."; console.log(data); })
        .catch(error => { console.error('Error:', error); document.getElementById('status').innerText = "Error: " + error.message; });
    }
  </script>
</body>
</html>
)rawliteral";

// ===================================================================================
//                             FUNCTION DECLARATIONS
// ===================================================================================
// --- Robot Control Functions ---
void updateTurning();
void performTimedTurn(bool isTurnRight, int angle); // <-- ADD THIS LINE
void checkI2CDevices();
void stopMotors();
void stopDevice();
void emergencyStop();
void setMotorState(int leftMotor_dir, int rightMotor_dir);
void setMotorPWM(int leftPWM, int rightPWM, bool applyCalibration);
int getSpeed(AsyncWebServerRequest *request);
float getAccurateHeading();
void updateTurning();

// --- Sensor & Task Functions ---
void setup_rpm_pcnt_new(pcnt_unit_handle_t *unit, int pulse_pin);
void handleRPMCalculation();
void obstacleMonitorTask(void *pvParameters);

// --- Voice Command Processing ---
void handleAudioUpload(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final);
TranscriptionResult transcribeAudioWithGroq(const uint8_t* audioData, size_t audioSize);
String getLlamaCommandFromGroq(String text);
void executeLlamaCommand(String jsonCommand);


// ===================================================================================
//                                CORE ROBOT FUNCTIONS
// ===================================================================================


void performTimedTurn(bool isTurnRight, int angle) {
  lcd.clear();
  lcd.print("Timed Turn:");
  lcd.setCursor(0, 1);
  lcd.printf("%d deg", angle);

  // Set motor direction based on turn
  if (isTurnRight) {
    setMotorState(MOTOR_FORWARD, MOTOR_BACKWARD);
  } else {
    setMotorState(MOTOR_BACKWARD, MOTOR_FORWARD);
  }

  // Calculate duration and apply power
  long turnDuration = angle * MS_PER_DEGREE_OF_TURN;
  setMotorPWM(TURN_PWM, TURN_PWM, false);
  delay(turnDuration); // Blocking delay for the turn duration

  // Stop motors and return to idle
  stopDevice();
  lcd.clear();
  lcd.print("Turn Complete");
}


void setMotorState(int leftMotor_dir, int rightMotor_dir) {
  digitalWrite(IN3_PIN, leftMotor_dir == MOTOR_BACKWARD ? HIGH : LOW);
  digitalWrite(IN4_PIN, leftMotor_dir == MOTOR_FORWARD  ? HIGH : LOW);
  digitalWrite(IN1_PIN, rightMotor_dir == MOTOR_FORWARD ? HIGH : LOW);
  digitalWrite(IN2_PIN, rightMotor_dir == MOTOR_BACKWARD ? HIGH : LOW);
}

void setMotorPWM(int leftPWM, int rightPWM, bool applyCalibration) {
  int finalLeftPWM = leftPWM;
  if (applyCalibration && (currentState == MOVING_FWD || currentState == MOVING_BWD)) {
    finalLeftPWM += motorCalibOffset;
  }
  
  finalLeftPWM = constrain(finalLeftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  ledc_set_duty(pwm_mode, left_pwm_ch, finalLeftPWM);
  ledc_update_duty(pwm_mode, left_pwm_ch);

  ledc_set_duty(pwm_mode, right_pwm_ch, rightPWM);
  ledc_update_duty(pwm_mode, right_pwm_ch);
}

void stopMotors() {
  setMotorState(MOTOR_STOP, MOTOR_STOP);
  setMotorPWM(0, 0, false);
}

void stopDevice() {
  currentSpeed = 0;
  stopMotors();
  currentState = IDLE;
}

void emergencyStop() {
  if (currentState != IDLE) {
    Serial.println("!!! EMERGENCY STOP ACTIVATED !!!");
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("EMERGENCY STOP!");
    lcd.setCursor(0, 1); lcd.print("Obstacle Found");
    stopDevice();
  }
}

int getSpeed(AsyncWebServerRequest *request) {
    int speed = 200;
    if (request->hasParam("speed")) speed = request->getParam("speed")->value().toInt();
    return constrain(speed, 150, 255);
}

float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, 30000);
  return (duration == 0) ? 999.0 : (duration * 0.0343 / 2.0);
}

void obstacleMonitorTask(void *pvParameters) {
  Serial.println("Obstacle monitoring task started on Core 1.");
  for (;;) {
    logicalBackDistance = readUltrasonic(BACK_TRIG_PIN, BACK_ECHO_PIN);
    logicalFrontDistance = readUltrasonic(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
    if ((currentState == MOVING_FWD && logicalFrontDistance <= OBSTACLE_THRESHOLD_CM) ||
        (currentState == MOVING_BWD && logicalBackDistance <= OBSTACLE_THRESHOLD_CM)) {
      emergencyStop();
    }
    vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL));
  }
}

// --- Add this complete function definition anywhere before setup() ---
void checkI2CDevices() {
  Serial.println("Scanning I2C bus...");
  byte error, address;
  int nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
    Serial.println("Expected: 0x27 (LCD) and 0x0D (Compass)");
  }
}

float getAccurateHeading() {
    compass.read();
    float heading = compass.getAzimuth();
    return (heading < 0 || heading >= 360) ? -1.0 : heading;
}

float calculateAngularDistance(float current, float target) {
    float diff = target - current;
    while (diff <= -180) diff += 360;
    while (diff > 180) diff -= 360;
    return diff;
}

int calculateTurnSpeed(float remainingAngle, int baseSpeed) {
    float absAngle = abs(remainingAngle);
    if (absAngle <= HEADING_TOLERANCE) return 0;
    if (absAngle < SLOW_DOWN_ANGLE) return map(absAngle, 0, SLOW_DOWN_ANGLE, MIN_TURN_PWM, baseSpeed);
    return constrain(baseSpeed, MIN_TURN_PWM, DEFAULT_TURN_PWM);
}

void updateTurning() {
    if (currentState != TURNING) return;
    unsigned long currentTime = millis();
    if (currentTime - turnStartTime > TURN_TIMEOUT) {
        stopDevice(); lcd.clear(); lcd.print("Turn Timeout!"); Serial.println("Turn Timeout!");
        return;
    }
    if (currentTime - lastCompassRead >= COMPASS_READ_INTERVAL) {
        lastCompassRead = currentTime;
        float currentHeading = getAccurateHeading();
        if (currentHeading < 0) return;
        float remainingAngle = calculateAngularDistance(currentHeading, turnTargetHeading);
        int dynamicSpeed = calculateTurnSpeed(remainingAngle, turnSpeed);
        if (dynamicSpeed == 0) {
            stopDevice(); lcd.clear(); lcd.print("Turn Complete!");
            lcd.setCursor(0, 1); lcd.printf("Final: %.1f", currentHeading); Serial.println("Turn Complete.");
        } else {
            setMotorPWM(dynamicSpeed, dynamicSpeed, false);
            lcd.setCursor(0, 1); lcd.printf("C:%.1f T:%.1f ", currentHeading, turnTargetHeading);
        }
    }
}

void setup_rpm_pcnt_new(pcnt_unit_handle_t *unit, int pulse_pin) {
  // Configure the Pulse Counter Unit
  pcnt_unit_config_t unit_config = {
    .low_limit = -32768,
    .high_limit = 32767,
    .flags = { .accum_count = 1 }, // Enable accumulation
  };
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, unit));

  // Configure the Channel
  pcnt_chan_config_t chan_config = {
    .edge_gpio_num = pulse_pin,
    .level_gpio_num = -1,
  };
  pcnt_channel_handle_t pcnt_chan = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(*unit, &chan_config, &pcnt_chan));

  // Set edge actions: Increment on Positive Edge, Do Nothing (Hold) on Negative Edge
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
  
  // Enable, clear, and start the counter
  ESP_ERROR_CHECK(pcnt_unit_enable(*unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(*unit));
  ESP_ERROR_CHECK(pcnt_unit_start(*unit));
}

void handleRPMCalculation() {
  if (millis() - lastRpmReadTime >= RPM_READ_INTERVAL) {
    int l_pulse = 0, r_pulse = 0;
    pcnt_unit_get_count(left_pcnt_unit, &l_pulse);
    pcnt_unit_get_count(right_pcnt_unit, &r_pulse);
    pcnt_unit_clear_count(left_pcnt_unit);
    pcnt_unit_clear_count(right_pcnt_unit);
    leftRPM = (l_pulse * (60000 / RPM_READ_INTERVAL)) / PULSES_PER_REVOLUTION;
    rightRPM = (r_pulse * (60000 / RPM_READ_INTERVAL)) / PULSES_PER_REVOLUTION;
    lastRpmReadTime = millis();
  }
}

void connectWiFi() {
  Serial.print("Connecting to WiFi...");
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Connecting WiFi");
  WiFi.mode(WIFI_STA); WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500); Serial.print(".");
    lcd.setCursor(attempts % 16, 1); lcd.print(".");
    attempts++;
  }
  if(WiFi.status() == WL_CONNECTED){
    Serial.println("\nWiFi Connected!"); Serial.print("IP Address: "); Serial.println(WiFi.localIP());
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("IP Address:"); lcd.setCursor(0, 1); lcd.print(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect."); lcd.clear(); lcd.setCursor(0, 0); lcd.print("WiFi Failed!");
  }
}

// ===================================================================================
//                                SETUP
// ===================================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Robot Starting...");
  delay(1000);

  // --- I2C DIAGNOSTIC ---
  lcd.clear();
  lcd.print("Scanning I2C...");
  checkI2CDevices(); // This will print results to Serial Monitor
  delay(2000);

  Serial.println("Setting up RPM sensors (New API)...");
  setup_rpm_pcnt_new(&left_pcnt_unit, LEFT_RPM_SENSOR_PIN);
  setup_rpm_pcnt_new(&right_pcnt_unit, RIGHT_RPM_SENSOR_PIN);

  Serial.println("Initializing compass...");
  lcd.clear();
  lcd.print("Init Compass...");
  compass.init();
  
  pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT); pinMode(IN4_PIN, OUTPUT);

  Serial.println("Configuring PWM Channels (New API)...");
  ledc_timer_config_t ledc_timer = { .speed_mode = pwm_mode, .duty_resolution = LEDC_TIMER_8_BIT, .timer_num = pwm_timer, .freq_hz = PWM_FREQ, .clk_cfg = LEDC_AUTO_CLK };
  ledc_timer_config(&ledc_timer);
  ledc_channel_config_t left_ledc_channel = { .gpio_num = ENA_PIN, .speed_mode = pwm_mode, .channel = left_pwm_ch, .intr_type = LEDC_INTR_DISABLE, .timer_sel = pwm_timer, .duty = 0, .hpoint = 0 };
  ledc_channel_config(&left_ledc_channel);
  ledc_channel_config_t right_ledc_channel = { .gpio_num = ENB_PIN, .speed_mode = pwm_mode, .channel = right_pwm_ch, .intr_type = LEDC_INTR_DISABLE, .timer_sel = pwm_timer, .duty = 0, .hpoint = 0 };
  ledc_channel_config(&right_ledc_channel);
  
  pinMode(FRONT_TRIG_PIN, OUTPUT); pinMode(FRONT_ECHO_PIN, INPUT);
  pinMode(BACK_TRIG_PIN, OUTPUT); pinMode(BACK_ECHO_PIN, INPUT);
  
  stopDevice();
  connectWiFi();

  // --- Web Server Handlers ---
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ request->send(200, "text/html", index_html); });
  
  server.on("/forward", HTTP_GET, [](AsyncWebServerRequest *request){ 
    currentSpeed = getSpeed(request); currentState = MOVING_FWD;
    setMotorState(MOTOR_FORWARD, MOTOR_FORWARD); request->send(200, "text/plain", "OK"); 
  });
  
  server.on("/backward", HTTP_GET, [](AsyncWebServerRequest *request){ 
    currentSpeed = getSpeed(request); currentState = MOVING_BWD;
    setMotorState(MOTOR_BACKWARD, MOTOR_BACKWARD); request->send(200, "text/plain", "OK"); 
  });

  server.on("/turn_left", HTTP_GET, [](AsyncWebServerRequest *request){
    int angle = 90; if(request->hasParam("angle")) angle = request->getParam("angle")->value().toInt();
    executeLlamaCommand("{\"command\":\"turn_left\", \"angle\":" + String(angle) + "}");
    request->send(200, "text/plain", "OK");
  });

  server.on("/turn_right", HTTP_GET, [](AsyncWebServerRequest *request){
    int angle = 90; if(request->hasParam("angle")) angle = request->getParam("angle")->value().toInt();
    executeLlamaCommand("{\"command\":\"turn_right\", \"angle\":" + String(angle) + "}");
    request->send(200, "text/plain", "OK");
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    stopDevice(); lcd.clear(); lcd.setCursor(0,0); lcd.print("Stopped.");
    request->send(200, "text/plain", "OK");
  });

  server.on("/upload", HTTP_POST, 
    [](AsyncWebServerRequest *request){
        request->send(200, "application/json", "{\"message\":\"Processing command...\"}");
    }, handleAudioUpload);

  server.begin();
  xTaskCreatePinnedToCore(obstacleMonitorTask, "ObstacleMonitor", 4096, NULL, 1, NULL, 1);
}

// ===================================================================================
//                           VOICE COMMAND PROCESSING
// ===================================================================================

void handleAudioUpload(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {
    static uint8_t* audioBuffer = NULL;
    static size_t bufferSize = 0;

    if (!index) {
        if(audioBuffer) { free(audioBuffer); audioBuffer = NULL; }
        Serial.printf("Audio Upload Start: %s\n", filename.c_str());
        audioBuffer = (uint8_t*)malloc(len);
        if (!audioBuffer) { Serial.println("Failed to allocate memory"); return; }
        memcpy(audioBuffer, data, len);
        bufferSize = len;
    } else {
        uint8_t* newBuffer = (uint8_t*)realloc(audioBuffer, bufferSize + len);
        if (!newBuffer) {
            Serial.println("Failed to reallocate memory");
            if (audioBuffer) free(audioBuffer);
            audioBuffer = NULL; return;
        }
        audioBuffer = newBuffer;
        memcpy(audioBuffer + bufferSize, data, len);
        bufferSize += len;
    }

    if (final && audioBuffer) {
        Serial.printf("Upload Complete. Total size: %u bytes\n", bufferSize);
        lcd.clear(); lcd.print("Transcribing...");
        
        // NEW: Get the result object instead of just a string
        TranscriptionResult txResult = transcribeAudioWithGroq(audioBuffer, bufferSize);
        
        free(audioBuffer); audioBuffer = NULL;

        // NEW: Check if the error string is empty
        if (txResult.error.isEmpty()) {
            // SUCCESS PATH
            lcd.clear(); lcd.print("Getting Cmd...");
            String llamaCommand = getLlamaCommandFromGroq(txResult.text);
            if (llamaCommand.length() > 0) {
                executeLlamaCommand(llamaCommand);
            } else { 
                lcd.clear(); lcd.print("LLM Error"); 
            }
        } else {
            // FAILURE PATH: Display the specific error
            lcd.clear(); 
            lcd.print("Tx Error:");
            lcd.setCursor(0, 1);
            lcd.print(txResult.error); // Display the specific error on the LCD
            Serial.println("Transcription Error: " + txResult.error);
        }
    }
}

TranscriptionResult transcribeAudioWithGroq(const uint8_t* audioData, size_t audioSize) {
    TranscriptionResult result; // Create a result object to be returned
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;

    if (http.begin(client, whisper_api_url)) {
        http.addHeader("Authorization", "Bearer " + String(groq_api_key));
        String boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
        http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);

        String model_name = "whisper-large-v3";
        String start_body = "--" + boundary + "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"command.webm\"\r\nContent-Type: audio/webm\r\n\r\n";
        String middle_body = "\r\n--" + boundary + "\r\nContent-Disposition: form-data; name=\"model\"\r\n\r\n" + model_name;
        String end_body = "\r\n--" + boundary + "--\r\n";
        
        size_t totalLength = start_body.length() + audioSize + middle_body.length() + end_body.length();
        
        uint8_t* postBuffer = (uint8_t*) malloc(totalLength);
        if(!postBuffer) {
            Serial.println("Failed to allocate memory for POST buffer");
            http.end();
            result.error = "Out of Memory";
            return result; // Return the result object with the error
        }

        size_t offset = 0;
        memcpy(postBuffer + offset, start_body.c_str(), start_body.length()); offset += start_body.length();
        memcpy(postBuffer + offset, audioData, audioSize); offset += audioSize;
        memcpy(postBuffer + offset, middle_body.c_str(), middle_body.length()); offset += middle_body.length();
        memcpy(postBuffer + offset, end_body.c_str(), end_body.length());

        int httpCode = http.POST(postBuffer, totalLength);
        free(postBuffer);

        if (httpCode > 0) {
            String payload = http.getString();
            if (httpCode == HTTP_CODE_OK) {
                JsonDocument doc;
                deserializeJson(doc, payload);
                result.text = doc["text"].as<String>(); // Success: set the text field
                Serial.printf("Transcription: %s\n", result.text.c_str());
            } else {
                Serial.printf("[Whisper] Error: %d | Payload: %s\n", httpCode, payload.c_str());
                result.error = "API Error " + String(httpCode); // Failure: set the error field
            }
        } else {
            Serial.printf("[Whisper] POST failed: %s\n", http.errorToString(httpCode).c_str());
            result.error = "Request Failed"; // Failure: set the error field
        }
        http.end();
    } else {
      Serial.println("[Whisper] Connection failed!");
      result.error = "Connect Failed"; // Failure: set the error field
    }
    return result; // Return the fully populated result object
}

String getLlamaCommandFromGroq(String text) {
    String command = "";
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;

    if (http.begin(client, llama_api_url)) {
        http.addHeader("Authorization", "Bearer " + String(groq_api_key));
        http.addHeader("Content-Type", "application/json");

        JsonDocument doc;
        doc["model"] = "llama3-8b-8192";
        JsonArray messages = doc["messages"].to<JsonArray>();
        JsonObject msg_system = messages.add<JsonObject>();
        msg_system["role"] = "system";
        msg_system["content"] = "You are a robot controller. Respond ONLY with a JSON object. The JSON must have a 'command' key. Valid commands: 'forward', 'backward', 'turn_left', 'turn_right', 'stop'. For turns, you can add an 'angle' key (e.g., 90). If no angle is specified for a turn, default to 45.";
        JsonObject msg_user = messages.add<JsonObject>();
        msg_user["role"] = "user";
        msg_user["content"] = text;
        doc["response_format"]["type"] = "json_object";

        String requestBody;
        serializeJson(doc, requestBody);
        
        int httpCode = http.POST(requestBody);
        if (httpCode > 0) {
            String payload = http.getString();
            if (httpCode == HTTP_CODE_OK) {
                JsonDocument responseDoc;
                deserializeJson(responseDoc, payload);
                command = responseDoc["choices"][0]["message"]["content"].as<String>();
                Serial.printf("Llama Command: %s\n", command.c_str());
            } else {
                Serial.printf("[Llama] Error: %d | Payload: %s\n", httpCode, payload.c_str());
            }
        } else {
            Serial.printf("[Llama] POST failed: %s\n", http.errorToString(httpCode).c_str());
        }
        http.end();
    }
    return command;
}

void executeLlamaCommand(String jsonCommand) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonCommand);
    if (error) {
        Serial.printf("JSON Parse Error: %s\n", error.c_str());
        lcd.clear(); lcd.print("JSON Error"); return;
    }
    const char* cmd = doc["command"];
    if (!cmd) {
        Serial.println("Command key missing in JSON");
        lcd.clear(); lcd.print("Invalid Cmd"); return;
    }
    String commandStr = String(cmd);
    int angle = doc["angle"] | 45;

    Serial.printf("Executing: %s, Angle: %d\n", commandStr.c_str(), angle);
    lcd.clear(); lcd.printf("Cmd: %s", commandStr.c_str());
    if (commandStr.startsWith("turn")) { lcd.printf(" %d", angle); }

    if (commandStr == "forward") {
        currentSpeed = 255; // Use maximum speed for voice command
        currentState = MOVING_FWD;
        setMotorState(MOTOR_FORWARD, MOTOR_FORWARD);
    } else if (commandStr == "backward") {
        currentSpeed = 255; // Use maximum speed for voice command
        currentState = MOVING_BWD;
        setMotorState(MOTOR_BACKWARD, MOTOR_BACKWARD);
    } else if (commandStr == "stop") {
        stopDevice();
    } else if (commandStr == "turn_left" || commandStr == "turn_right") {
        if (currentState != IDLE) stopDevice();
        delay(100);
        
        float startHeading = getAccurateHeading();

        if (startHeading < 0) {
            Serial.println("Compass error! Falling back to timed turn.");
            lcd.clear(); lcd.print("Compass ERR");
            delay(500);
            performTimedTurn((commandStr == "turn_right"), angle);
        } else {
            if (commandStr == "turn_left") {
                turnTargetHeading = fmod(startHeading - angle + 360.0, 360.0);
                setMotorState(MOTOR_BACKWARD, MOTOR_FORWARD);
            } else {
                turnTargetHeading = fmod(startHeading + angle, 360.0);
                setMotorState(MOTOR_FORWARD, MOTOR_BACKWARD);
            }
            turnSpeed = TURN_PWM;
            turnStartTime = millis();
            currentState = TURNING;
            setMotorPWM(turnSpeed, turnSpeed, false);
        }
    } else {
        Serial.printf("Unknown command: %s\n", commandStr.c_str());
        lcd.clear(); lcd.print("Unknown Cmd");
    }
}

// ===================================================================================
//                                      MAIN LOOP
// ===================================================================================
void loop() {
  if (WiFi.status() != WL_CONNECTED && millis() - lastReconnectAttempt > 10000) {
    lastReconnectAttempt = millis();
    connectWiFi();
  }
  
  handleRPMCalculation();
  updateTurning();

  switch(currentState) {
    case IDLE:      break;
    case MOVING_FWD:
    case MOVING_BWD:
      setMotorPWM(currentSpeed, currentSpeed, true);
      break;
    case TURNING:   break;
  }
  
  delay(10); 
}