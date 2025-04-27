#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <math.h>

ESP8266WebServer server(80);  // Web server on port 80

String word_command = "-O-";
String car_dir = "S";
String car_speed = "50";  
String joint = "2"; 
int A1 = 90, A2 = 90, A3 = 90, A4 = 90, A5 = 90, A6 = 90 , A7 = 90; // Initialize arm positions

void handleButton() {
  word_command = server.arg("btn");
  server.send(200, "text/plain", "OK");
}

void handleButtonRelease() {
  word_command = "S";  // Stop command
  server.send(200, "text/plain", "OK");
}

void handleSpeed() {
  car_speed = server.arg("value");
  server.send(200, "text/plain", "OK");
}

void handleRoot() {
    static const char MAIN_page[] PROGMEM = R"rawliteral(
      <html>
    <style>
        :root {
            --bg-color: #f0f0f0;
            --panel-color: #e0e0e0;
            --button-bg: #fff;
            --button-text: #000;
            --joystick-knob: #888;
        }
        body {
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            background-color: var(--bg-color);
        }
        .extra_parent {
            display: flex;
            position: absolute;
            top: 15vh;
            justify-content: center;
            align-items: center;
            flex-direction: row;
            width: 100vw;
        }
        .extra_parent1 {
            display: flex;
            position: absolute;
            flex-direction: column;
            justify-content: center;
            left: 8.5vw;
            align-items: center;
            flex-direction: row;
        }
        .extra_parent2 {
            display: flex;
            position: absolute;
            flex-direction: column;
            justify-content: center;
            right: 8.5vw;
            align-items: center;
            flex-direction: row;
        }
        .container2 {
            position: absolute;
            right: 2vw;
            top: 30%;
            display: flex;
            flex-direction: column;
            position: absolute;
            justify-content: center;
            align-items: center;
            margin: 10px;
        }
        button {
            width: 10vw;
            height: 20vh;
            font-size: 2em;
            margin: 5px;
            border-radius: 2vw;
            background: var(--button-bg);
            color: var(--button-text);
            box-shadow: 0 2px 8px #aaa;
        }
        button:active,
        button.pressed {
            background-color: #cccccc;
            transform: scale(0.96);
            box-shadow: 0 2px #666;
        }
        .Stop {
            background-color: red !important;
            color: #fff;
        }
        .joystick-container {
            position: absolute;
            left: 10vw;
            top: 65%;
            transform: translateY(-50%);
            width: 20vw;           /* Increased size */
            height: 40vh;          /* Increased size */
            min-width: 120px;
            min-height: 120px;
            max-width: 350px;
            max-height: 350px;
            background: #000;
            border-radius: 2vw;
            display: flex;
            justify-content: center;
            align-items: center;
            box-shadow: 0 2px 8px #aaa;
            touch-action: none;
        }
        .joystick-knob {
            width: 7vw;            /* Bigger knob */
            height: 14vh;
            min-width: 60px;
            min-height: 60px;
            max-width: 140px;
            max-height: 140px;
            background: red;       /* Red knob */
            border: 4px solid #ffffff; /* Black border in light mode */
            border-radius: 50%;
            position: absolute;
            left: 50%;
            top: 50%;
            transform: translate(-50%, -50%);
            transition: background 0.1s, border-color 0.1s;
            touch-action: none;
            box-shadow: 0 10px 15px #a19898;
        }
        /* Theme toggle button */
        .theme-toggle-btn {
            position: fixed;
            left: 2vw;
            bottom: 2vh;
            width: 48px;
            height: 48px;
            border-radius: 50%;
            background: #222;
            color: #fff;
            border: none;
            font-size: 1.5em;
            box-shadow: 0 2px 8px #aaa;
            cursor: pointer;
            z-index: 1000;
            opacity: 0.7;
            transition: background 0.2s, opacity 0.2s;
        }
        .theme-toggle-btn:hover {
            background: #444;
            opacity: 1;
        }
        /* Dark mode variables */
        body.dark {
            --bg-color: #181818;
            --panel-color: #232323;
            --button-bg: #222;
            --button-text: #fff;
        }
        body.dark .joystick-container {
            background: #fff;
            box-shadow: 0 2px 8px #111;
        }
        body.dark .joystick-knob {
            background: red; /* Red knob in dark mode too */
            border: 4px solid #000; /* Black border in dark mode */
            box-shadow: 0 2px 8px #111;
        }
        body.dark button {
            background: #222;
            color: #fff;
            box-shadow: 0 2px 8px #111;
        }
        body.dark .Stop {
            background-color: red !important;
            color: #fff;
        }
        body.dark button:active,
        body.dark button.pressed {
            background-color: #cccccc !important;   /* Same as light mode */
            color: #fff;
            transform: scale(0.96);
            box-shadow: 0 2px #666;
            transition: background 0.08s, transform 0.08s, box-shadow 0.08s;
        }
        .speed-control {
            display: flex;
            align-items: center;
            justify-content: center;
            margin: 0 15px;
            background: var(--panel-color);
            padding: 10px;
            border-radius: 10px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.2);
        }
        .slider {
            -webkit-appearance: none;
            width: 100px;
            height: 15px;
            border-radius: 8px;
            background: #d3d3d3;
            outline: none;
            margin: 0 10px;
        }
        .slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 25px;
            height: 25px;
            border-radius: 50%;
            background: red;
            cursor: pointer;
            border: 2px solid white;
        }
        .slider::-moz-range-thumb {
            width: 25px;
            height: 25px;
            border-radius: 50%;
            background: red;
            cursor: pointer;
            border: 2px solid white;
        }
        #speedValue {
            min-width: 40px;
            text-align: center;
            font-weight: bold;
        }
        body.dark .speed-control {
            background: var(--panel-color);
            color: white;
        }
        body.dark .slider {
            background: #555;
        }
        @media (orientation: portrait) {
            body {
                display: flex;
                flex-direction: column;
                justify-content: center;
                align-items: center;
                height: 100vh;
                background-color: var(--bg-color);
                position: relative;
                overflow: hidden;
            }
            .extra_parent {
            display: flex;
            position: absolute;
            top: 5vh;
            justify-content: center;
            align-items: center;
            flex-direction: row;
            width: 100vw;
            }
            .extra_parent1 {
            display: flex;
            position: absolute;
            flex-direction: column;
            justify-content: center;
            left: 5vw;
            align-items: center;
            flex-direction: row;
            }
            .extra_parent2 {
                display: flex;
                position: absolute;
                flex-direction: column;
                justify-content: center;
                right: 5vw;
                align-items: center;
                flex-direction: row;
            }
            .RL {
                display: flex;
                flex-direction: row;
                justify-content: center;
                align-items: center;
            }
            .container2 {
                position: absolute;
                left: 50%;
                top: 18%;
                transform: translateX(-50%);
                display: flex;
                flex-direction: column;
                justify-content: center;
                align-items: center;
                margin: 10px;
                height: fit-content;
                width: fit-content;
            }
            button {
                width: 20vw;
                height: 20vw;
                font-size: 2em;
                border-radius: 1.2em;
                box-sizing: border-box;
                white-space: nowrap;
            }
            .joystick-container {
                position: absolute;
                left: 50%;
                top: 80vh;
                transform: translate(-50%, -50%);
                width: 30vh;
                aspect-ratio: 1/1;   /* Enforce perfect circle */
                height: auto;        /* Let aspect-ratio control height */
                min-width: 120px;
                min-height: 120px;
                max-width: 350px;
                max-height: 350px;
            }
            .joystick-knob {
                width: 14vh;
                aspect-ratio: 1/1;
                height: auto;
                min-width: 60px;
                min-height: 60px;
                max-width: 140px;
                max-height: 140px;
            }
            .parent{
                display: flex;
                justify-content: center;
                align-items: center;
                flex-direction: column;
                height: 100vh;
                width: 100vw;
            }
        }
    </style>
</head>
<body>
    <div class="extra_parent">
        <div class="extra_parent1">
            <button id="extra1">1</button>
            <button id="extra2">2</button>
        </div>
        <div class="speed-control">
            <label for="speedSlider">Speed:</label>
            <input type="range" id="speedSlider" min="0" max="100" value="50" class="slider">
            <span id="speedValue">50%</span>
        </div>
        <div class="extra_parent2">
            <button id="extra3">3</button>
            <button id="extra4">4</button>
        </div>
    </div>
    <button class="theme-toggle-btn" id="themeToggle" title="Toggle dark mode">-D-</button>
<div class="parent">
    <div class="joystick-container" id="joystick">
        <div class="joystick-knob" id="joystickKnob"></div>
    </div>
    <div class="container2">
        <button id="Up">U</button>
        <div class="RL">
           <button id="Close"> > < </button>
           <button id="Stop1" class="Stop">S</button>
           <button id="Open"> < > </button>
        </div>
        <button id="Down">D</button>
    </div>
</div>

<script>
    // Flag to track if user has interacted with the page
    let userInteracted = false;

    // Mark that user has interacted with the page
    document.addEventListener('pointerdown', function() {
        userInteracted = true;
    });

    // Helper to send commands to ESP8266
    function sendCommand(cmd) {
        // Only send commands if user has interacted with the page
        if (!userInteracted && cmd !== "S") {
            console.log("Ignoring initial command: " + cmd);
            return;
        }
        fetch('/button?btn=' + encodeURIComponent(cmd));
    }
    
    function sendRelease() {
        if (!userInteracted) return;
        fetch('/button_release');
    }
    
    function sendSpeed(value) {
        if (!userInteracted) return;
        fetch('/speed?value=' + encodeURIComponent(value));
    }

    // Joystick logic
    const joystick = document.getElementById('joystick');
    const knob = document.getElementById('joystickKnob');
    let dragging = false;
    let center = { x: 0, y: 0 };
    let maxDist = 0;
    let lastDir = "S";

    function getDirection(dx, dy) {
        const angle = Math.atan2(dy, dx) * 180 / Math.PI;
        if (Math.sqrt(dx*dx + dy*dy) < maxDist * 0.3) return "S"; // Center = Stop
        if (angle >= -45 && angle < 45) return "R";
        if (angle >= 45 && angle < 135) return "B";
        if (angle >= -135 && angle < -45) return "F";
        return "L";
    }

    function setKnob(x, y) {
        knob.style.left = x + 'px';
        knob.style.top = y + 'px';
    }

    function resetKnob() {
        setKnob(joystick.clientWidth/2, joystick.clientHeight/2);
    }

    joystick.addEventListener('pointerdown', function(e) {
        dragging = true;
        const rect = joystick.getBoundingClientRect();
        center = { x: rect.width/2, y: rect.height/2 };
        maxDist = rect.width/2 - knob.offsetWidth/2;
        moveKnob(e);
    });
    window.addEventListener('pointermove', function(e) {
        if (!dragging) return;
        moveKnob(e);
    });
    window.addEventListener('pointerup', function(e) {
        if (!dragging) return;
        dragging = false;
        resetKnob();
        lastDir = "S";
        sendCommand("S");
    });

    function moveKnob(e) {
        const rect = joystick.getBoundingClientRect();
        let x = e.clientX - rect.left;
        let y = e.clientY - rect.top;
        let dx = x - center.x;
        let dy = y - center.y;
        const dist = Math.sqrt(dx*dx + dy*dy);
        if (dist > maxDist) {
            dx = dx * maxDist / dist;
            dy = dy * maxDist / dist;
        }
        setKnob(center.x + dx, center.y + dy);
        const dir = getDirection(dx, dy);
        if (dir !== lastDir) {
            lastDir = dir;
            sendCommand(dir);
        }
    }

    // Initialize knob position
    resetKnob();

    // Add tactile effect for all buttons on pointer events and send commands
    document.addEventListener('DOMContentLoaded', function () {
        document.querySelectorAll('button').forEach(function (btn) {
            btn.addEventListener('pointerdown', function () {
                btn.classList.add('pressed');
                // Map button to command
                let cmd = "";
                if (btn.id === "Up") cmd = "U";
                else if (btn.id === "Down") cmd = "D";
                else if (btn.id === "Close") cmd = "C";
                else if (btn.id === "Open") cmd = "O";
                else if (btn.id === "extra1") cmd = "1";
                else if (btn.id === "extra2") cmd = "2";
                else if (btn.id === "extra3") cmd = "3";
                else if (btn.id === "extra4") cmd = "4";
                else if (btn.classList.contains("Stop")) cmd = "S";
                else cmd = btn.textContent.trim();
                sendCommand(cmd);
            });
            btn.addEventListener('pointerup', function () {
                btn.classList.remove('pressed');
                sendRelease();
            });
            btn.addEventListener('pointerleave', function () {
                btn.classList.remove('pressed');
                sendRelease();
            });
        });

        const speedSlider = document.getElementById('speedSlider');
        const speedValue = document.getElementById('speedValue');
        
        // Initialize with default value
        speedValue.textContent = speedSlider.value + "%";
        
        // Update when slider is moved
        speedSlider.addEventListener('input', function() {
            const value = speedSlider.value;
            speedValue.textContent = value + "%";
            sendSpeed(value);
        });
        
        // Send the initial speed value
        sendSpeed(speedSlider.value);
    });

    // Theme toggle logic
    const themeToggle = document.getElementById('themeToggle');
    themeToggle.addEventListener('click', function () {
        document.body.classList.toggle('dark');
        // Change icon
        themeToggle.textContent = document.body.classList.contains('dark') ? '-O-' : '-D-';
    });
</script>
      </html>
    )rawliteral";
    server.send(200, "text/html", MAIN_page);
  }

void carCommandUpdate(){
    if (word_command == "F") {
        car_dir = "F";  // Forward
    } else if (word_command == "B") {
        car_dir = "B";  // Backward
    } else if (word_command == "L") {
        car_dir = "L";  // Left
    } else if (word_command == "R") {
        car_dir = "R";  // Right
    } else if (word_command == "S") {
        car_dir = "S";   // Stop command
    }  
}

void jointUpdate(){
    if (word_command == "1") {
        joint = "1"; 
    } else if (word_command == "2"){
        joint = "2";
    } else if (word_command == "3"){
        joint = "3";
    } else if (word_command == "4"){
        joint = "4";
    } 
}

void servoCommandUpdate(){
    // Common controls for gripper (joint 1)
    if (word_command == "C"){
        A1 = constrain(A1 + 1, 0, 180);  // Increment with upper limit
    } else if (word_command == "O"){
        A1 = constrain(A1 - 1, 0, 180);  // Decrement with lower limit
    }
    
    // Joint-specific up/down controls
    if (word_command == "U" || word_command == "D") {
        if (joint == "1"){
            if (word_command == "U"){
                A2 = constrain(A2 + 1, 0, 180);
            } else if (word_command == "D"){
                A2 = constrain(A2 - 1, 0, 180);
            }
        } else if (joint == "2"){
            if (word_command == "U"){
                A3 = constrain(A3 + 1, 0, 180);
            } else if (word_command == "D"){
                A3 = constrain(A3 - 1, 0, 180);
            }
        } else if (joint == "3"){
            if (word_command == "U"){
                A4 = constrain(A4 + 1, 0, 180);
                A5 = constrain(A5 - 1, 0, 180);
            } else if (word_command == "D"){
                A4 = constrain(A4 - 1, 0, 180);
                A5 = constrain(A5 + 1, 0, 180);
            }
        } else if (joint == "4"){
            if (word_command == "U"){
                A6 = constrain(A6 + 1, 0, 180);
                A7 = constrain(A7 - 1, 0, 180);
            } else if (word_command == "D"){
                A6 = constrain(A6 - 1, 0, 180);
                A7 = constrain(A7 + 1, 0, 180);
            }
        }
    }
    delay(50);
}

void kinematics(){
    
}

void variableValueUpdate(){
    carCommandUpdate();
    servoCommandUpdate();  
}

void sendCommandToArduino(){
    String Command = "";
    Command = "<" + car_dir + "," + car_speed + "," + 
              "j1," + String(A1) + "," + 
              "j2," + String(A2) + "," + 
              "j3," + String(A3) + "," + 
              "j4," + String(A4) + "," + 
              "j5," + String(A5) + "," + 
              "j6," + String(A6) + "," +
              "j7," + String(A7) + ">"; 
    Serial.println(Command);  // Send command to Arduino via Serial
}

void setup() {
  Serial.begin(57600);
  // Set up the ESP8266 as an Access Point (AP)
  WiFi.softAP("Alice", "access_alice");  // Create Wi-Fi AP with SSID and password
  
  // Print the IP address of ESP8266 to Serial Monitor
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
  
  // Handle button presses for controlling LEDs
  server.on("/", HTTP_GET, handleRoot);
  server.on("/button", HTTP_GET, handleButton);
  server.on("/button_release", HTTP_GET, handleButtonRelease);
  server.on("/speed", HTTP_GET, handleSpeed);

  // Start the web server
  server.begin();
  Serial.println("HTTP server started");
  
  // Send a valid initial command to Arduino to prevent "invalid command" errors
  delay(1000); // Wait for serial to be ready
  sendCommandToArduino();
}

void loop() {
  server.handleClient();  // Handle client requests

  jointUpdate();

  variableValueUpdate();  // Update variable values based on commands received
  
  sendCommandToArduino();  // Send command to Arduino
  
  delay(10);  // Delay to prevent flooding the serial port
}