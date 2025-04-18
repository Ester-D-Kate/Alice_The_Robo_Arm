#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

ESP8266WebServer server(80);  // Web server on port 80

void handleButton() {
  String btn = server.arg("btn");
  String msg;
  if (btn == "Forward") msg = "F";
  else if (btn == "Backward") msg = "B";
  else if (btn == "Left") msg = "L";
  else if (btn == "Right") msg = "R";
  else if (btn == "Stop") msg = "S";
  else msg = btn;
  Serial.println(msg);
  server.send(200, "text/plain", "OK");
}

void handleButtonRelease() {
  Serial.println("S");
  server.send(200, "text/plain", "OK");
}

// Home page route: Display the UI for controlling LEDs
void handleRoot() {
  static const char MAIN_page[] PROGMEM = R"rawliteral(
    <html>
    <style>
        body {
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            background-color: #f0f0f0;
        }
        .container1 {
            display: flex;
            flex-direction: column;
            position: absolute;
            justify-content: center;
            align-items: center;
            margin: 10px;
            margin-right: 62vw;
        }
        .container2 {
            display: flex;
            flex-direction: column;
            position: absolute;
            justify-content: center;
            align-items: center;
            margin: 10px;
            margin-left: 53vw;
        }
        button {
            width: 10vw;
            height: 20vh;
            font-size: 2em;
            margin: 5px;
            border-radius: 2vw;
        }
        button:active {
            background-color: #cccccc;
            transform: scale(0.96);
            box-shadow: 0 2px #666;
        }
        .Stop {
            background-color: red;
        }
    </style>
    
    <style>
        @media (orientation: portrait) {
            body {
                display: flex;
                flex-direction: row;
                justify-content: center;
                align-items: center;
                height: 100vh;
                background-color: #f0f0f0;
            }
            .container1 {
                margin: 50vw;
            }
            .RL {
                display: flex;
                flex-direction: row;
                justify-content: center;
                align-items: center;
            }
            .container2 {
                display: none !important;
            }
            button {
                width: 20vw;
                height: 20vh;
                font-size: 2em;
            }
        }
    </style>
</head>
<body>
    <div class="container1">
        <button id="Forward">F</button>
        <div class="RL">
        <button id="Right">R</button>
        <button id="Stop1" class="Stop">S</button>
        <button id="Left">L</button>
        </div>
        <button id="Backward">B</button>
        </div>
        <div class="container2">
            <button id="Forward">F</button>
            <div>
            <button id="Right">R</button>
            <button id="Stop1" class="Stop">S</button>
            <button id="Left">L</button>
            </div>
            <button id="Backward">B</button>
            </div>
<script>
    // Send button press/release to ESP8266
    document.addEventListener('DOMContentLoaded', function () {
        document.querySelectorAll('button').forEach(function (btn) {
            btn.addEventListener('pointerdown', function () {
                btn.classList.add('pressed');
                let btnType = btn.id;
                if (btn.classList.contains("Stop")) btnType = "Stop";
                fetch(`/button?btn=${btnType}`);
            });
            btn.addEventListener('pointerup', function () {
                btn.classList.remove('pressed');
                fetch('/button_release');
            });
            btn.addEventListener('pointerleave', function () {
                btn.classList.remove('pressed');
            });
        });
    });
</script>
    </html>
  )rawliteral";
  server.send(200, "text/html", MAIN_page);
}

// Set the ESP8266 in AP mode and start the web server
void setup() {
  Serial.begin(115200);
  // Set up the ESP8266 as an Access Point (AP)
  WiFi.softAP("Alice", "amritsar");  // Create Wi-Fi AP with SSID and password
  
  // Print the IP address of ESP8266 to Serial Monitor
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
  
  // Handle button presses for controlling LEDs
  server.on("/", HTTP_GET, handleRoot);
  server.on("/button", HTTP_GET, handleButton);
  server.on("/button_release", HTTP_GET, handleButtonRelease);

  // Start the web server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();  // Handle client requests
}

