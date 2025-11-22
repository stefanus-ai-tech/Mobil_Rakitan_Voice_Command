#ifndef WEBINTERFACE_H
#define WEBINTERFACE_H

#include <Arduino.h>

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP32 Groq Robot</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; text-align: center; margin-top: 20px; background-color: #2c3e50; color: #ecf0f1; }
    h1 { color: #3498db; }
    .btn-container { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px; max-width: 320px; margin: 20px auto; }
    .btn { background-color: #3498db; border: none; color: white; padding: 20px; font-size: 18px; cursor: pointer; border-radius: 10px; user-select: none; }
    .btn:active { background-color: #2980b9; transform: scale(0.98); }
    .stop { background-color: #e74c3c; font-weight: bold; }
    .voice-btn { background-color: #9b59b6; grid-column: 1 / -1; margin-top: 10px; }
    .voice-btn.recording { background-color: #e74c3c; animation: pulse 1s infinite; }
    @keyframes pulse { 0% { box-shadow: 0 0 0 0 rgba(231, 76, 60, 0.7); } 70% { box-shadow: 0 0 0 20px rgba(231, 76, 60, 0); } 100% { box-shadow: 0 0 0 0 rgba(231, 76, 60, 0); } }
    #status { margin-top: 20px; font-size: 1.2em; color: #f1c40f; }
    .slider-container { margin: 20px auto; max-width: 300px; }
    input[type=range] { width: 100%; }
  </style>
</head>
<body>
  <h1>ESP32 Robot v3</h1>
  <div class="btn-container">
    <div></div><button class="btn" onmousedown="send('forward')" onmouseup="send('stop')" ontouchstart="send('forward')" ontouchend="send('stop')">FWD</button><div></div>
    <button class="btn" onmousedown="send('turn_left')" onmouseup="send('stop')" ontouchstart="send('turn_left')" ontouchend="send('stop')">LEFT</button>
    <button class="btn stop" onclick="send('stop')">STOP</button>
    <button class="btn" onmousedown="send('turn_right')" onmouseup="send('stop')" ontouchstart="send('turn_right')" ontouchend="send('stop')">RIGHT</button>
    <div></div><button class="btn" onmousedown="send('backward')" onmouseup="send('stop')" ontouchstart="send('backward')" ontouchend="send('stop')">BWD</button>
    <button id="voiceBtn" class="btn voice-btn" onmousedown="startRec()" onmouseup="stopRec()" ontouchstart="startRec()" ontouchend="stopRec()">Hold for Voice</button>
  </div>
  <div class="slider-container">
    <label>Speed: <span id="spdVal">200</span></label>
    <input type="range" min="100" max="255" value="200" id="slider" oninput="document.getElementById('spdVal').innerText=this.value">
  </div>
  <p id="status">Ready.</p>

  <script>
    function send(cmd) {
      let spd = document.getElementById('slider').value;
      fetch('/' + cmd + '?speed=' + spd).then(r => document.getElementById('status').innerText = "Cmd: " + cmd);
    }

    let mediaRecorder, chunks = [];
    async function startRec() {
      try {
        let stream = await navigator.mediaDevices.getUserMedia({ audio: true });
        mediaRecorder = new MediaRecorder(stream);
        chunks = [];
        mediaRecorder.ondataavailable = e => chunks.push(e.data);
        mediaRecorder.onstop = uploadAudio;
        mediaRecorder.start();
        document.getElementById('voiceBtn').classList.add('recording');
        document.getElementById('status').innerText = "Listening...";
      } catch(e) { alert("Mic Error (Use HTTPS or localhost): " + e); }
    }
    function stopRec() {
      if (mediaRecorder && mediaRecorder.state === "recording") mediaRecorder.stop();
      document.getElementById('voiceBtn').classList.remove('recording');
    }
    function uploadAudio() {
      document.getElementById('status').innerText = "Processing...";
      let blob = new Blob(chunks, { type: 'audio/wav' });
      let fd = new FormData();
      fd.append("audio", blob, "cmd.wav");
      fetch("/upload", { method: "POST", body: fd })
        .then(r => r.json())
        .then(d => document.getElementById('status').innerText = d.message)
        .catch(e => document.getElementById('status').innerText = "Error: " + e);
    }
  </script>
</body>
</html>
)rawliteral";

#endif