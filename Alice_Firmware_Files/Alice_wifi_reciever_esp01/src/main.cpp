#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// **UNCOMMENT TO FORCE CONFIG MODE FOR TESTING**
//#define FORCE_CONFIG_MODE

// MQTT Configuration
const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = "";

// Robot configuration
String ssid_stored = "";
String password_stored = "";
bool configMode = false;
int wifiRetries = 0;
const int maxWifiRetries = 5;

// **ENHANCED CONTROL SYSTEM**
enum ControlMode {
  MANUAL_CONTINUOUS,  // Press/hold for continuous movement
  POSITION_ABSOLUTE   // Set exact servo positions
};

struct ServoControl {
  int currentPosition;
  int targetPosition;
  int minPosition;
  int maxPosition;
  bool isMoving;
  unsigned long lastMoveTime;
};

// Robot control variables
String word_command = "S";
String car_dir = "S";
String car_speed = "50";
String joint = "2";
ControlMode controlMode = MANUAL_CONTINUOUS;

// **SERVO CONTROL SYSTEM - 5 servos**
ServoControl servos[5] = {
  {90, 90, 0, 180, false, 0},  // A1 - Claw
  {90, 90, 0, 180, false, 0},  // A2 - Wrist rotation
  {90, 90, 0, 180, false, 0},  // A3 - Claw up/down
  {90, 90, 0, 180, false, 0},  // A4 - Full arm motor 1
  {90, 90, 0, 180, false, 0}   // A5 - Full arm motor 2 (mirrored)
};

// MQTT Topics
const char* car_command_topic = "alice/zero/car/command";
const char* arm_command_topic = "alice/zero/arm/command";
const char* arm_position_topic = "alice/zero/arm/position";
const char* status_topic = "alice/zero/robot/status";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
ESP8266WebServer server(80);

unsigned long lastStatusTime = 0;
unsigned long commandTimeout = 0;
unsigned long lastMoveTime = 0;

// **FUNCTION DECLARATIONS**
void clearEEPROM();
void loadCredentials();
void saveCredentials(String ssid, String password);
bool testWiFiConnection(String ssid, String password);
void connectToWiFi();
void startConfigMode();
void setupWebServer();
void setupMQTT();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void handleCarCommand(JsonDocument& doc);
void handleArmCommand(JsonDocument& doc);
void handlePositionCommand(JsonDocument& doc);
void startManualMovement(String cmd);
void stopAllMovement();
void updateRobotControl();
void updateManualMovement();
void updatePositionMovement();
void carCommandUpdate();
void sendCommandToArduino();
void publishStatus();
void handleRoot();
void handleScan();
void handleConnect();
void handleStatus();
void handleNotFound();
void handleClearWiFi();

void setup() {
  Serial.begin(57600);
  EEPROM.begin(512);
  
  Serial.println("========================================");
  Serial.println("    ZERO ROBOT ENHANCED CONTROL");
  Serial.println("========================================");
  
  // **CHECK FOR FORCE CONFIG MODE**
  #ifdef FORCE_CONFIG_MODE
    Serial.println("FORCE_CONFIG_MODE enabled - clearing EEPROM...");
    clearEEPROM();
    Serial.println("EEPROM cleared! Starting config mode...");
    startConfigMode();
    return;
  #endif
  
  // **CHECK FOR MANUAL RESET BUTTON (GPIO 0)**
  pinMode(0, INPUT_PULLUP);
  delay(100);
  
  if (digitalRead(0) == LOW) {
    Serial.println("RESET BUTTON PRESSED - clearing WiFi credentials...");
    clearEEPROM();
    Serial.println("WiFi credentials cleared! Starting config mode...");
    startConfigMode();
    return;
  }
  
  loadCredentials();
  
  if (ssid_stored.length() > 0 && password_stored.length() > 0) {
    Serial.println("Found stored WiFi: " + ssid_stored);
    
    if (testWiFiConnection(ssid_stored, password_stored)) {
      connectToWiFi();
    } else {
      Serial.println("Stored WiFi invalid. Starting config mode...");
      startConfigMode();
    }
  } else {
    Serial.println("No WiFi credentials. Starting config mode...");
    startConfigMode();
  }
}

void loop() {
  if (configMode) {
    server.handleClient();
  } else {
    if (!mqttClient.connected()) {
      reconnectMQTT();
    }
    mqttClient.loop();
    
    // **ENHANCED UPDATE SYSTEM**
    updateRobotControl();
    sendCommandToArduino();
    
    // Status updates every 1 second for real-time feedback
    if (millis() - lastStatusTime > 1000) {
      publishStatus();
      lastStatusTime = millis();
    }
    
    // Auto-stop safety
    if (millis() - commandTimeout > 2000 && word_command != "S") {
      word_command = "S";
      stopAllMovement();
      Serial.println("Auto-stop: No commands received");
    }
    
    // Monitor WiFi health
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected! Attempting reconnection...");
      wifiRetries++;
      
      if (wifiRetries >= maxWifiRetries) {
        Serial.println("WiFi connection failed multiple times. Starting config mode...");
        startConfigMode();
      } else {
        connectToWiFi();
      }
    }
  }
  
  delay(20); // Reduced for smoother servo movement
}

// **CLEAR EEPROM FUNCTION**
void clearEEPROM() {
  Serial.println("Clearing EEPROM...");
  for (int i = 0; i < 512; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  ssid_stored = "";
  password_stored = "";
  Serial.println("EEPROM cleared successfully!");
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Zero Robot WiFi Setup</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 20px; background: #f0f0f0; }
        .container { max-width: 400px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
        h1 { color: #333; text-align: center; margin-bottom: 30px; }
        .form-group { margin-bottom: 15px; position: relative; }
        label { display: block; margin-bottom: 5px; font-weight: bold; }
        input, select { width: 100%; padding: 10px; border: 1px solid #ddd; border-radius: 5px; font-size: 16px; box-sizing: border-box; }
        button { width: 100%; padding: 12px; background: #007bff; color: white; border: none; border-radius: 5px; font-size: 16px; cursor: pointer; margin-top: 10px; }
        button:hover { background: #0056b3; }
        .scan-btn { background: #28a745; }
        .scan-btn:hover { background: #1e7e34; }
        .clear-btn { background: #dc3545; }
        .clear-btn:hover { background: #c82333; }
        .status { padding: 10px; border-radius: 5px; margin: 10px 0; }
        .error { background: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }
        .success { background: #d4edda; color: #155724; border: 1px solid #c3e6cb; }
        .loading { color: #007bff; }
        .password-toggle { position: absolute; right: 10px; top: 32px; cursor: pointer; color: #666; user-select: none; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Zero Robot WiFi Setup</h1>
        
        <div class="form-group">
            <button class="scan-btn" onclick="scanNetworks()">Scan for Networks</button>
        </div>
        
        <form id="wifiForm" onsubmit="return connectWiFi(event)">
            <div class="form-group">
                <label for="ssid">WiFi Network:</label>
                <select id="ssid" name="ssid" required>
                    <option value="">Select a network...</option>
                </select>
            </div>
            
            <div class="form-group">
                <label for="password">Password:</label>
                <input type="password" id="password" name="password" placeholder="Enter WiFi password">
                <span class="password-toggle" onclick="togglePassword()">Show</span>
            </div>
            
            <button type="submit">Connect Robot</button>
        </form>
        
        <div class="form-group">
            <button class="clear-btn" onclick="clearWiFi()">Clear Stored WiFi</button>
        </div>
        
        <div id="status"></div>
    </div>

    <script>
        function scanNetworks() {
            document.getElementById('status').innerHTML = '<div class="status loading">Scanning for networks...</div>';
            
            fetch('/scan')
                .then(response => response.json())
                .then(networks => {
                    const select = document.getElementById('ssid');
                    select.innerHTML = '<option value="">Select a network...</option>';
                    
                    networks.forEach(network => {
                        const option = document.createElement('option');
                        option.value = network.ssid;
                        option.textContent = network.ssid + ' (' + network.rssi + ' dBm)';
                        select.appendChild(option);
                    });
                    
                    document.getElementById('status').innerHTML = '<div class="status success">Found ' + networks.length + ' networks</div>';
                })
                .catch(error => {
                    document.getElementById('status').innerHTML = '<div class="status error">Error scanning: ' + error + '</div>';
                });
        }

        function togglePassword() {
            const passwordField = document.getElementById('password');
            const toggleBtn = document.querySelector('.password-toggle');
            
            if (passwordField.type === 'password') {
                passwordField.type = 'text';
                toggleBtn.textContent = 'Hide';
            } else {
                passwordField.type = 'password';
                toggleBtn.textContent = 'Show';
            }
        }

        function connectWiFi(event) {
            event.preventDefault();
            
            const ssid = document.getElementById('ssid').value;
            const password = document.getElementById('password').value;
            
            if (!ssid) {
                document.getElementById('status').innerHTML = '<div class="status error">Please select a network</div>';
                return false;
            }
            
            document.getElementById('status').innerHTML = '<div class="status loading">Connecting to ' + ssid + '...</div>';
            
            const formData = new FormData();
            formData.append('ssid', ssid);
            formData.append('password', password);
            
            fetch('/connect', {
                method: 'POST',
                body: formData
            })
            .then(response => response.json())
            .then(result => {
                if (result.success) {
                    document.getElementById('status').innerHTML = '<div class="status success">Connected successfully!<br>IP: ' + result.ip + '<br>Robot is now online!</div>';
                    setTimeout(() => {
                        window.location.reload();
                    }, 3000);
                } else {
                    document.getElementById('status').innerHTML = '<div class="status error">Connection failed: ' + result.message + '</div>';
                }
            })
            .catch(error => {
                document.getElementById('status').innerHTML = '<div class="status error">Error: ' + error + '</div>';
            });
            
            return false;
        }

        function clearWiFi() {
            if (confirm('Are you sure you want to clear stored WiFi credentials?')) {
                document.getElementById('status').innerHTML = '<div class="status loading">Clearing WiFi credentials...</div>';
                
                fetch('/clear', {
                    method: 'POST'
                })
                .then(response => response.json())
                .then(result => {
                    if (result.success) {
                        document.getElementById('status').innerHTML = '<div class="status success">WiFi credentials cleared! Device will restart in config mode.</div>';
                        setTimeout(() => {
                            window.location.reload();
                        }, 2000);
                    } else {
                        document.getElementById('status').innerHTML = '<div class="status error">Failed to clear credentials: ' + result.message + '</div>';
                    }
                })
                .catch(error => {
                    document.getElementById('status').innerHTML = '<div class="status error">Clear error: ' + error + '</div>';
                });
            }
        }

        // Auto-scan on page load
        window.onload = function() {
            scanNetworks();
        };
    </script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

// **WIFI CONFIGURATION FUNCTIONS**
void loadCredentials() {
  Serial.println("Loading WiFi credentials from EEPROM...");
  
  // Read SSID length
  int ssidLength = EEPROM.read(0);
  if (ssidLength > 0 && ssidLength < 100) {
    for (int i = 0; i < ssidLength; i++) {
      ssid_stored += char(EEPROM.read(1 + i));
    }
  }
  
  // Read password length
  int passwordLength = EEPROM.read(100);
  if (passwordLength > 0 && passwordLength < 100) {
    for (int i = 0; i < passwordLength; i++) {
      password_stored += char(EEPROM.read(101 + i));
    }
  }
  
  Serial.println("SSID: " + ssid_stored);
  Serial.println("Password: [Hidden]");
}

void saveCredentials(String ssid, String password) {
  Serial.println("Saving WiFi credentials to EEPROM...");
  
  // Clear previous data
  for (int i = 0; i < 512; i++) {
    EEPROM.write(i, 0);
  }
  
  // Save SSID
  EEPROM.write(0, ssid.length());
  for (int i = 0; i < ssid.length(); i++) {
    EEPROM.write(1 + i, ssid[i]);
  }
  
  // Save password
  EEPROM.write(100, password.length());
  for (int i = 0; i < password.length(); i++) {
    EEPROM.write(101 + i, password[i]);
  }
  
  EEPROM.commit();
  Serial.println("Credentials saved successfully!");
}

bool testWiFiConnection(String ssid, String password) {
  Serial.println("Testing WiFi connection...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());
  
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi test successful!");
    return true;
  } else {
    Serial.println("\nWiFi test failed!");
    WiFi.disconnect();
    return false;
  }
}

void connectToWiFi() {
  Serial.println("Connecting to WiFi: " + ssid_stored);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid_stored.c_str(), password_stored.c_str());
  
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 40) {
    delay(500);
    Serial.print(".");
    timeout++;
    
    if (timeout >= 40) {
      Serial.println("\nWiFi connection timeout!");
      wifiRetries++;
      return;
    }
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.println("IP address: " + WiFi.localIP().toString());
    Serial.println("Signal strength: " + String(WiFi.RSSI()) + " dBm");
    
    wifiRetries = 0;
    configMode = false;
    setupMQTT();
  }
}

void startConfigMode() {
  Serial.println("========================================");
  Serial.println("    STARTING WIFI CONFIGURATION MODE");
  Serial.println("========================================");
  
  configMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ZeroRobot_Config", "12345678");
  
  IPAddress IP = WiFi.softAPIP();
  Serial.println("Configuration AP started");
  Serial.println("SSID: ZeroRobot_Config");
  Serial.println("Password: 12345678");
  Serial.println("IP address: " + IP.toString());
  Serial.println("Open browser and go to: http://" + IP.toString());
  Serial.println("========================================");
  
  setupWebServer();
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/scan", handleScan);
  server.on("/connect", handleConnect);
  server.on("/status", handleStatus);
  server.on("/clear", handleClearWiFi);
  server.onNotFound(handleNotFound);
  
  server.begin();
  Serial.println("Web server started on port 80");
}

void handleScan() {
  Serial.println("Scanning for WiFi networks...");
  int n = WiFi.scanNetworks();
  
  String json = "[";
  for (int i = 0; i < n; i++) {
    if (i > 0) json += ",";
    json += "{";
    json += "\"ssid\":\"" + WiFi.SSID(i) + "\",";
    json += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
    json += "\"encryption\":\"" + String(WiFi.encryptionType(i)) + "\"";
    json += "}";
  }
  json += "]";
  
  server.send(200, "application/json", json);
}

void handleConnect() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  
  Serial.println("Attempting to connect to: " + ssid);
  
  if (testWiFiConnection(ssid, password)) {
    saveCredentials(ssid, password);
    
    String response = "{\"success\":true,\"message\":\"Connected successfully\",\"ip\":\"" + WiFi.localIP().toString() + "\"}";
    server.send(200, "application/json", response);
    
    delay(2000);
    ssid_stored = ssid;
    password_stored = password;
    
    // Stop the SoftAP and web server
    server.stop();
    WiFi.softAPdisconnect(true);
    
    // Connect to the new WiFi
    connectToWiFi();
  } else {
    String response = "{\"success\":false,\"message\":\"Failed to connect to WiFi network\"}";
    server.send(200, "application/json", response);
  }
}

void handleClearWiFi() {
  Serial.println("Clearing WiFi credentials via web interface...");
  clearEEPROM();
  
  String response = "{\"success\":true,\"message\":\"WiFi credentials cleared\"}";
  server.send(200, "application/json", response);
  
  delay(1000);
  ESP.restart();
}

void handleStatus() {
  String json = "{";
  json += "\"status\":\"" + String(WiFi.status() == WL_CONNECTED ? "connected" : "disconnected") + "\",";
  json += "\"ssid\":\"" + WiFi.SSID() + "\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"rssi\":" + String(WiFi.RSSI());
  json += "}";
  
  server.send(200, "application/json", json);
}

void handleNotFound() {
  server.send(404, "text/plain", "Page not found");
}

void setupMQTT() {
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);
  
  Serial.println("Connecting to MQTT broker...");
  reconnectMQTT();
}

void reconnectMQTT() {
  int attempts = 0;
  while (!mqttClient.connected() && attempts < 5) {
    Serial.print("Attempting MQTT connection... ");
    
    String clientId = "ZeroRobot-" + String(random(0xffff), HEX);
    
    if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("CONNECTED!");
      
      mqttClient.subscribe(car_command_topic);
      mqttClient.subscribe(arm_command_topic);
      mqttClient.subscribe(arm_position_topic);
      
      Serial.println("========================================");
      Serial.println("    ROBOT READY - DUAL CONTROL MODE");
      Serial.println("Manual: " + String(arm_command_topic));
      Serial.println("Position: " + String(arm_position_topic));
      Serial.println("========================================");
      
      publishStatus();
      
    } else {
      Serial.println("FAILED (rc=" + String(mqttClient.state()) + ")");
      attempts++;
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  String message = "";
  
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("MQTT [" + topicStr + "]: " + message);
  commandTimeout = millis();
  
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    if (message.length() <= 2) {
      word_command = message;
      controlMode = MANUAL_CONTINUOUS;
      Serial.println("Manual command: " + word_command);
      return;
    }
  } else {
    if (topicStr == car_command_topic) {
      handleCarCommand(doc);
    } else if (topicStr == arm_command_topic) {
      handleArmCommand(doc);
    } else if (topicStr == arm_position_topic) {
      handlePositionCommand(doc);
    }
  }
}

void handleCarCommand(JsonDocument& doc) {
  if (!doc["cmd"].isNull()) {
    String cmd = doc["cmd"].as<String>();
    if (cmd == "F" || cmd == "B" || cmd == "L" || cmd == "R" || cmd == "S") {
      word_command = cmd;
      Serial.println("Car command: " + cmd);
    }
  }
  
  if (!doc["speed"].isNull()) {
    car_speed = doc["speed"].as<String>();
    Serial.println("Car speed: " + car_speed);
  }
}

void handleArmCommand(JsonDocument& doc) {
  if (!doc["cmd"].isNull()) {
    String cmd = doc["cmd"].as<String>();
    
    if (cmd == "U" || cmd == "D" || cmd == "C" || cmd == "O" || 
        cmd == "1" || cmd == "2" || cmd == "3" || cmd == "4" || cmd == "S") {
      word_command = cmd;
      controlMode = MANUAL_CONTINUOUS;
      Serial.println("Manual arm command: " + cmd);
      
      if (cmd == "S") {
        stopAllMovement();
      } else {
        startManualMovement(cmd);
      }
    }
  }
  
  if (!doc["joint"].isNull()) {
    joint = doc["joint"].as<String>();
  }
}

void handlePositionCommand(JsonDocument& doc) {
  Serial.println("Position command received");
  controlMode = POSITION_ABSOLUTE;
  word_command = "S";
  
  if (!doc["A1"].isNull()) {
    servos[0].targetPosition = constrain(doc["A1"].as<int>(), 0, 180);
    servos[0].isMoving = true;
    Serial.println("A1 target: " + String(servos[0].targetPosition));
  }
  
  if (!doc["A2"].isNull()) {
    servos[1].targetPosition = constrain(doc["A2"].as<int>(), 0, 180);
    servos[1].isMoving = true;
    Serial.println("A2 target: " + String(servos[1].targetPosition));
  }
  
  if (!doc["A3"].isNull()) {
    servos[2].targetPosition = constrain(doc["A3"].as<int>(), 0, 180);
    servos[2].isMoving = true;
    Serial.println("A3 target: " + String(servos[2].targetPosition));
  }
  
  if (!doc["A4"].isNull()) {
    servos[3].targetPosition = constrain(doc["A4"].as<int>(), 0, 180);
    servos[3].isMoving = true;
    Serial.println("A4 target: " + String(servos[3].targetPosition));
  }
  
  if (!doc["A5"].isNull()) {
    servos[4].targetPosition = constrain(doc["A5"].as<int>(), 0, 180);
    servos[4].isMoving = true;
    Serial.println("A5 target: " + String(servos[4].targetPosition));
  }
}

void startManualMovement(String cmd) {
  if (cmd == "U") {
    servos[3].isMoving = true;
    servos[4].isMoving = true;
  } else if (cmd == "D") {
    servos[3].isMoving = true;
    servos[4].isMoving = true;
  } else if (cmd == "C") {
    servos[0].isMoving = true;
  } else if (cmd == "O") {
    servos[0].isMoving = true;
  } else if (cmd == "1") {
    servos[1].isMoving = true;
  } else if (cmd == "4") {
    servos[1].isMoving = true;
  } else if (cmd == "2") {
    servos[2].isMoving = true;
  } else if (cmd == "3") {
    servos[2].isMoving = true;
  }
}

void stopAllMovement() {
  for (int i = 0; i < 5; i++) {
    servos[i].isMoving = false;
  }
  Serial.println("All servo movement stopped");
}

void updateRobotControl() {
  carCommandUpdate();
  
  if (millis() - lastMoveTime > 50) {
    if (controlMode == MANUAL_CONTINUOUS) {
      updateManualMovement();
    } else if (controlMode == POSITION_ABSOLUTE) {
      updatePositionMovement();
    }
    lastMoveTime = millis();
  }
}

void updateManualMovement() {
  int step = 2;
  
  if (word_command == "1" && servos[1].isMoving) {
    servos[1].currentPosition = constrain(servos[1].currentPosition - step, 0, 180);
  } else if (word_command == "4" && servos[1].isMoving) {
    servos[1].currentPosition = constrain(servos[1].currentPosition + step, 0, 180);
  } else if (word_command == "2" && servos[2].isMoving) {
    servos[2].currentPosition = constrain(servos[2].currentPosition + step, 0, 180);
  } else if (word_command == "3" && servos[2].isMoving) {
    servos[2].currentPosition = constrain(servos[2].currentPosition - step, 0, 180);
  } else if (word_command == "U") {
    if (servos[3].isMoving) servos[3].currentPosition = constrain(servos[3].currentPosition + step, 0, 180);
    if (servos[4].isMoving) servos[4].currentPosition = constrain(servos[4].currentPosition - step, 0, 180);
  } else if (word_command == "D") {
    if (servos[3].isMoving) servos[3].currentPosition = constrain(servos[3].currentPosition - step, 0, 180);
    if (servos[4].isMoving) servos[4].currentPosition = constrain(servos[4].currentPosition + step, 0, 180);
  } else if (word_command == "C" && servos[0].isMoving) {
    servos[0].currentPosition = constrain(servos[0].currentPosition + step, 0, 180);
  } else if (word_command == "O" && servos[0].isMoving) {
    servos[0].currentPosition = constrain(servos[0].currentPosition - step, 0, 180);
  }
}

void updatePositionMovement() {
  int step = 3;
  bool anyMoving = false;
  
  for (int i = 0; i < 5; i++) {
    if (servos[i].isMoving) {
      if (servos[i].currentPosition < servos[i].targetPosition) {
        servos[i].currentPosition = min(servos[i].currentPosition + step, servos[i].targetPosition);
        anyMoving = true;
      } else if (servos[i].currentPosition > servos[i].targetPosition) {
        servos[i].currentPosition = max(servos[i].currentPosition - step, servos[i].targetPosition);
        anyMoving = true;
      } else {
        servos[i].isMoving = false;
      }
    }
  }
  
  if (!anyMoving && controlMode == POSITION_ABSOLUTE) {
    controlMode = MANUAL_CONTINUOUS;
    Serial.println("Position movement complete - switching to manual mode");
  }
}

void carCommandUpdate() {
  if (word_command == "F") car_dir = "F";
  else if (word_command == "B") car_dir = "B";
  else if (word_command == "L") car_dir = "L";
  else if (word_command == "R") car_dir = "R";
  else if (word_command == "S") car_dir = "S";
}

void sendCommandToArduino() {
  String Command = "<" + car_dir + "," + car_speed + "," + 
                  "j1," + String(servos[0].currentPosition) + "," +
                  "j2," + String(servos[1].currentPosition) + "," +
                  "j3," + String(servos[2].currentPosition) + "," +
                  "j4," + String(servos[3].currentPosition) + "," +
                  "j5," + String(servos[4].currentPosition) + "," +
                  "j6,90,j7,90>";
  Serial.println(Command);
}

void publishStatus() {
  JsonDocument doc;
  doc["device_id"] = "zero_robot";
  doc["status"] = "online";
  doc["timestamp"] = millis();
  doc["uptime"] = millis() / 1000;
  doc["control_mode"] = (controlMode == MANUAL_CONTINUOUS) ? "manual" : "position";
  
  JsonObject car = doc["car"].to<JsonObject>();
  car["direction"] = car_dir;
  car["speed"] = car_speed.toInt();
  car["last_command"] = word_command;
  
  JsonObject arm = doc["arm"].to<JsonObject>();
  arm["active_joint"] = joint.toInt();
  
  JsonObject positions = arm["positions"].to<JsonObject>();
  positions["A1"] = servos[0].currentPosition;
  positions["A2"] = servos[1].currentPosition;
  positions["A3"] = servos[2].currentPosition;
  positions["A4"] = servos[3].currentPosition;
  positions["A5"] = servos[4].currentPosition;
  
  JsonObject movement = arm["movement"].to<JsonObject>();
  movement["A1"] = servos[0].isMoving;
  movement["A2"] = servos[1].isMoving;
  movement["A3"] = servos[2].isMoving;
  movement["A4"] = servos[3].isMoving;
  movement["A5"] = servos[4].isMoving;
  
  JsonObject system = doc["system"].to<JsonObject>();
  system["free_heap"] = ESP.getFreeHeap();
  system["wifi_rssi"] = WiFi.RSSI();
  system["ip"] = WiFi.localIP().toString();
  
  String output;
  serializeJson(doc, output);
  mqttClient.publish(status_topic, output.c_str());
}
