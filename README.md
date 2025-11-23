# ESP32 AI Voice Robot (Groq API + Core v3.0)

A modular, voice-controlled robot powered by the ESP32 and Groq Cloud AI. This project integrates modern ESP32 Arduino Core v3.0 APIs, Asynchronous Web Server, and a robust hardware abstraction layer.

## 🚀 Key Features

- **Modern ESP32 Core v3.0 Compliance**: Uses the new `ledcAttach` and `ledcWrite` APIs, abandoning deprecated drivers.
- **AI Voice Control**: Records audio via browser, processes Speech-to-Text (Whisper) and Intent Recognition (Llama 4) via **Groq API**.
- **Modular Architecture**: Clean separation of concerns using `.h` and `.cpp` files.
- **Reactive Web UI**: Gamepad-style control and voice recorder hosted directly on the ESP32.
- **Safety First**: Dedicated obstacle avoidance loop running independently of the web server.

## 🛠 Hardware Requirements

- **Microcontroller**: ESP32 Dev Module (Doit ESP32 DEVKIT V1 recommended)
- **Motor Driver**: L298N
- **Sensors**: 2x HC-SR04 Ultrasonic Sensors (Front & Back)
- **Display**: LCD 1602 with I2C Backpack
- **Chassis**: 2WD or 4WD Robot Chassis
- **Power**: 2x 18650 Li-ion batteries (recommended)

## 📂 Module Architecture

The code is refactored into specific modules to ensure maintainability. Here is how they interact:

| Module | File(s) | Responsibility |
|--------|---------|----------------|
| **Main** | `*.ino` | **Orchestrator**. Initializes all modules and runs the safety loop to check sensors. |
| **Config** | `Config.h` | **Source of Truth**. Contains permanent pin definitions, API URLs, and global constants. |
| **Env** | `env.h` | **Security**. Stores WiFi credentials and API Keys (Not uploaded to Git). |
| **Motor** | `MotorControl` | **Hardware Layer**. Handles PWM (LEDC) generation and ultrasonic sensor readings. |
| **Network** | `NetworkManager` | **Connectivity**. Manages WiFi connection, Web Server, and serving the HTML UI. |
| **AI** | `GroqClient` | **Intelligence**. Handles HTTPS requests to Groq (Whisper for audio, Llama 4 for logic). |
| **Display** | `LCDHandler` | **Feedback**. Provides visual status updates to the user. |

## 📊 System Architecture

<img width="1298" height="842" alt="image" src="https://github.com/user-attachments/assets/b1089847-0fc0-4d2f-a247-89778bbcfa90" />

```
@startuml
!theme vibrant

title ESP32 Voice-Controlled Robot System

skinparam BackgroundColor FFFFFF
skinparam shadowing false
skinparam roundcorner 10

' User Interface
package "User Interface" {
    component [Web Browser] as UI
    note right of UI
    Fungsi:
    - Kontrol manual robot
    - Rekam perintah suara
    - Kirim audio ke ESP32
    end note
}

' ESP32 Core
package "ESP32 Firmware" {
    component [NetworkManager] as Net
    component [GroqClient] as AI
    component [MotorControl] as Motor
    component [LCDHandler] as LCD
    component [Main Loop] as Main
    database [Config Files] as Conf
}

' Cloud Services
cloud "Groq Cloud API" {
    component [Whisper STT] as STT
    component [Llama 4 LLM] as LLM
}

' Main Flow
UI --> Net : (1) Manual Control\nHTTP GET
UI --> Net : (2) Upload Audio\nHTTP POST

Net --> Motor : Direct Move
Net --> AI : Audio Buffer
Net ..> LCD : Update Status

AI --> STT : (3) POST Audio
STT --> AI : Return Text

AI --> LLM : (4) POST Text
LLM --> AI : Return JSON

AI --> Motor : (5) Execute Command
AI ..> LCD : Show Result

Main --> Motor : (6) Monitor Sensors
Motor --> Main : Distance Data
Main --> Motor : Emergency Stop

Conf --> Net : WiFi Config
Conf --> AI : API Key
Conf --> Motor : Pin Defs

legend bottom
Alur Kerja:
1. User kirim perintah (manual/suara)
2. Audio diproses oleh Whisper
3. Teks dikirim ke Llama 4
4. Robot eksekusi perintah
5. Sensor monitor keamanan
endlegend

@enduml
```



### Flow Explanation

1. **Manual Control**: The Web Interface sends HTTP GET requests directly to `NetworkManager`, which calls `MotorControl`.

2. **Voice Control**:
   - Audio is recorded in the browser and sent to `NetworkManager`
   - `NetworkManager` buffers the audio and passes it to `GroqClient`
   - `GroqClient` sends audio to **Whisper**, gets text, sends text to **Llama 4**, and receives a JSON command (e.g., `{"command": "turn_left", "angle": 90}`)
   - `GroqClient` parses the JSON and instructs `MotorControl` to move

3. **Safety Loop**: The `Main` loop constantly polls `MotorControl` for distance data. If an object is too close, it overrides all other commands and stops the robot.

## ⚙️ Installation & Setup

### 1. Clone the Repository

```bash
git clone https://github.com/stefanus-ai-tech/Mobil_Rakitan_Voice_Command
cd Mobil_Rakitan_Voice_Command
```

### 2. Configure Environment

Locate the file `env_template.h` and rename it to `env.h`:

```bash
cp env_template.h env.h
```

**IMPORTANT**: Add `env.h` to your `.gitignore` file to prevent leaking passwords.

Edit `env.h` with your credentials:

```cpp
const char* WIFI_SSID = "Your_WiFi_Name";
const char* WIFI_PASS = "Your_WiFi_Password";
const char* GROQ_API_KEY = "gsk_YourRealGroqKeyHere";
```

### 3. Pin Configuration

Check `Config.h` and ensure the pin definitions match your wiring.

> **Note**: The pins in the code are hardcoded for standard ESP32 layouts but can be adjusted in this file if necessary.

### 4. Install Required Libraries

Open Arduino IDE and install the following libraries via Library Manager:

- `ESPAsyncWebServer`
- `AsyncTCP`
- `ArduinoJson`
- `LiquidCrystal_I2C`

### 5. Upload to ESP32

1. Open `ESP32_Robot_Groq.ino` in Arduino IDE
2. Select Board: **ESP32 Dev Module**
3. Select the correct COM Port
4. Click Upload

## 🎮 Usage

### Manual Control

1. Connect to the WiFi network
2. Open the serial monitor to see the ESP32's IP address
3. Navigate to `http://<ESP32_IP_ADDRESS>` in your browser
4. Use the gamepad-style interface to control the robot

### Voice Control

> **Important**: Modern browsers require HTTPS for microphone access. Since ESP32 serves HTTP, you need to enable insecure origins:
> 
> **For Chrome/Edge:**
> 1. Navigate to `chrome://flags` (or `edge://flags`)
> 2. Search for "Insecure origins treated as secure"
> 3. Add `http://<ESP32_IP_ADDRESS>` (e.g., `http://192.168.1.100`)
> 4. Restart browser
> 
> **Alternative**: Access from `http://localhost` if using USB serial bridge

1. Click the microphone button on the web interface
2. Speak your command (e.g., "move forward", "turn left 90 degrees")
3. The robot will process your voice command and execute it

## 🔧 Troubleshooting

### Robot doesn't move
- Check motor driver connections
- Verify power supply is adequate (7-12V recommended)
- Check serial monitor for error messages

### WiFi connection fails
- Verify credentials in `env.h`
- Check if your router supports 2.4GHz (ESP32 doesn't support 5GHz)
- Try increasing `WIFI_CONNECT_TIMEOUT` in `Config.h`

### Voice commands not working
- Verify your Groq API key is valid
- Check internet connection stability
- Look for error messages in serial monitor
- Ensure microphone permissions are granted in browser

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the project
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📝 License

Distributed under the MIT License. See `LICENSE` for more information.

## 🙏 Acknowledgments

- [Groq](https://groq.com/) for providing fast AI inference API
- [ESP32 Community](https://github.com/espressif/arduino-esp32) for excellent hardware support
- All contributors who have helped improve this project

## ⚠️ Disclaimer

This project relies on the Groq API. Ensure you have a valid API Key and a stable internet connection for voice features to work.

---
