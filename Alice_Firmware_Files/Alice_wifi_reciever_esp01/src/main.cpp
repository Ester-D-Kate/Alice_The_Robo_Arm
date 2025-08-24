#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// EMQX Public Broker - FREE & UNLIMITED
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

// Robot control variables
String word_command = "S";
String car_dir = "S";
String car_speed = "50";
String joint = "2";

// **FIXED: Only 5 servos for your 4-joint + dual motor setup**
int A1 = 90, A2 = 90, A3 = 90, A4 = 90, A5 = 90;

// MQTT Topics - YOUR PERMANENT ROBOT URL
const char* car_command_topic = "alice/zero/car/command";
const char* arm_command_topic = "alice/zero/arm/command";
const char* status_topic = "alice/zero/robot/status";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
ESP8266WebServer server(80);

unsigned long lastStatusTime = 0;
unsigned long commandTimeout = 0;

// Function declarations
void connectToWiFi();
void startConfigMode();
void setupMQTT();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void handleCarCommand(JsonDocument& doc);
void handleArmCommand(JsonDocument& doc);
void publishStatus();
void handleConfigPage();
void handleSaveConfig();
void handleTestCredentials();
void saveCredentials();
void loadCredentials();
void clearCredentials();
void writeStringToEEPROM(int addr, String data);
String readStringFromEEPROM(int addr);
void carCommandUpdate();
void servoCommandUpdate();
void variableValueUpdate();
void sendCommandToArduino();
bool testWiFiConnection(String ssid, String password);
String sanitizeInput(String input);

void setup() {
  Serial.begin(57600);
  EEPROM.begin(512);
  
  Serial.println("========================================");
  Serial.println("        ZERO ROBOT STARTING");
  Serial.println("========================================");
  
  loadCredentials();
  
  // Check if we have stored credentials
  if (ssid_stored.length() > 0 && password_stored.length() > 0) {
    Serial.println("Found stored WiFi: " + ssid_stored);
    
    if (testWiFiConnection(ssid_stored, password_stored)) {
      connectToWiFi();
    } else {
      Serial.println("Stored WiFi invalid. Starting config mode...");
      wifiRetries++;
      if (wifiRetries >= maxWifiRetries) {
        clearCredentials(); // Clear bad credentials
        wifiRetries = 0;
      }
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
    
    variableValueUpdate();
    sendCommandToArduino();
    
    if (millis() - lastStatusTime > 3000) {
      publishStatus();
      lastStatusTime = millis();
    }
    
    // Auto-stop safety
    if (millis() - commandTimeout > 5000 && word_command != "S") {
      word_command = "S";
      Serial.println("Auto-stop: No commands received");
    }
    
    // Monitor WiFi health
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected! Attempting reconnection...");
      wifiRetries++;
      
      if (wifiRetries >= maxWifiRetries) {
        Serial.println("Max WiFi retries reached. Starting config mode...");
        clearCredentials(); // Clear bad credentials
        wifiRetries = 0;
        startConfigMode();
      } else {
        connectToWiFi();
      }
    }
  }
  
  delay(10);
}

bool testWiFiConnection(String ssid, String password) {
  Serial.println("Testing WiFi: " + ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi test successful!");
    WiFi.disconnect();
    return true;
  } else {
    Serial.println("\n✗ WiFi test failed!");
    WiFi.disconnect();
    return false;
  }
}

void connectToWiFi() {
  Serial.println("Connecting to WiFi: " + ssid_stored);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid_stored.c_str(), password_stored.c_str());
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n========================================");
    Serial.println("         WIFI CONNECTED!");
    Serial.println("Network: " + ssid_stored);
    Serial.println("IP: " + WiFi.localIP().toString());
    Serial.println("Signal: " + String(WiFi.RSSI()) + " dBm");
    Serial.println("========================================");
    
    configMode = false;
    wifiRetries = 0;
    setupMQTT();
  } else {
    Serial.println("\nWiFi connection failed!");
    wifiRetries++;
    startConfigMode();
  }
}

void setupMQTT() {
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);
  
  Serial.println("Connecting to EMQX MQTT broker...");
  reconnectMQTT();
}

void reconnectMQTT() {
  int attempts = 0;
  while (!mqttClient.connected() && attempts < 5) {
    Serial.print("Attempting MQTT connection... ");
    
    String clientId = "ZeroRobot-" + String(random(0xffff), HEX);
    
    if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("CONNECTED to EMQX!");
      
      mqttClient.subscribe(car_command_topic);
      mqttClient.subscribe(arm_command_topic);
      
      Serial.println("========================================");
      Serial.println("    ROBOT READY FOR MQTT COMMANDS!");
      Serial.println("Car topic: " + String(car_command_topic));
      Serial.println("Arm topic: " + String(arm_command_topic));
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
  
  // Parse JSON commands
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    // Handle simple string commands
    if (message.length() <= 2) {
      word_command = message;
      Serial.println("Simple command: " + word_command);
      return;
    }
  } else {
    // Handle JSON commands
    if (topicStr == car_command_topic) {
      handleCarCommand(doc);
    } else if (topicStr == arm_command_topic) {
      handleArmCommand(doc);
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
    if (cmd == "U" || cmd == "D" || cmd == "C" || cmd == "O") {
      word_command = cmd;
      Serial.println("Arm command: " + cmd);
    }
  }
  
  if (!doc["joint"].isNull()) {
    joint = doc["joint"].as<String>();
    Serial.println("Active joint: " + joint);
  }
}

void publishStatus() {
  JsonDocument doc;
  doc["device_id"] = "zero_robot";
  doc["status"] = "online";
  doc["timestamp"] = millis();
  doc["uptime"] = millis() / 1000;
  
  JsonObject car = doc["car"].to<JsonObject>();
  car["direction"] = car_dir;
  car["speed"] = car_speed.toInt();
  car["last_command"] = word_command;
  
  JsonObject arm = doc["arm"].to<JsonObject>();
  arm["active_joint"] = joint.toInt();
  
  // **FIXED: Only 5 servo positions for your setup**
  JsonObject servos = arm["positions"].to<JsonObject>();
  servos["A1"] = A1;  // Claw
  servos["A2"] = A2;  // Wrist rotation
  servos["A3"] = A3;  // Claw up/down
  servos["A4"] = A4;  // Full arm motor 1
  servos["A5"] = A5;  // Full arm motor 2 (mirrored)
  
  JsonObject system = doc["system"].to<JsonObject>();
  system["free_heap"] = ESP.getFreeHeap();
  system["wifi_rssi"] = WiFi.RSSI();
  system["ip"] = WiFi.localIP().toString();
  
  String output;
  serializeJson(doc, output);
  mqttClient.publish(status_topic, output.c_str());
}

void startConfigMode() {
  configMode = true;
  WiFi.mode(WIFI_AP);
  
  // **ZERO ACCESS POINT WITH YOUR SPECIFIED CREDENTIALS**
  bool apStarted = WiFi.softAP("zero", "E1s2t3e4r5", 1, false, 4);
  
  if (apStarted) {
    Serial.println("========================================");
    Serial.println("       CONFIGURATION MODE ACTIVE");
    Serial.println("Network: zero");
    Serial.println("Password: E1s2t3e4r5");
    Serial.println("Setup URL: http://192.168.4.1");
    Serial.println("========================================");
  } else {
    Serial.println("Failed to start Access Point!");
    delay(5000);
    ESP.restart();
  }
  
  server.on("/", handleConfigPage);
  server.on("/save", HTTP_POST, handleSaveConfig);
  server.on("/test", HTTP_POST, handleTestCredentials);
  server.begin();
}

void handleConfigPage() {
  String networks = "";
  int n = WiFi.scanNetworks();
  
  for (int i = 0; i < n && i < 10; i++) {
    if (WiFi.SSID(i).length() > 0) {
      networks += "<div class='network' onclick='selectNetwork(\"" + WiFi.SSID(i) + "\")'>";
      networks += WiFi.SSID(i);
      networks += " (" + String(WiFi.RSSI(i)) + " dBm)";
      if (WiFi.encryptionType(i) != ENC_TYPE_NONE) {
        networks += " [SECURED]";
      }
      networks += "</div>";
    }
  }
  
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>Zero Robot WiFi Setup</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body {font-family: Arial; margin: 20px; background: #f0f2f5;}";
  html += ".container {max-width: 400px; margin: 0 auto; background: white; padding: 30px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1);}";
  html += "h1 {text-align: center; color: #333;}";
  html += ".networks {max-height: 150px; overflow-y: auto; border: 1px solid #ddd; border-radius: 5px; margin: 15px 0;}";
  html += ".network {padding: 10px; cursor: pointer; border-bottom: 1px solid #eee;}";
  html += ".network:hover {background: #f8f9fa;}";
  html += "input {width: 100%; padding: 12px; margin: 8px 0; border: 2px solid #ddd; border-radius: 5px; box-sizing: border-box;}";
  html += ".btn {width: 100%; padding: 12px; margin: 10px 0; border: none; border-radius: 5px; font-size: 16px; cursor: pointer;}";
  html += ".btn-primary {background: #007cba; color: white;}";
  html += ".btn-test {background: #28a745; color: white;}";
  html += "#status {margin-top: 15px; padding: 10px; border-radius: 5px; display: none;}";
  html += ".success {background: #d4edda; color: #155724;}";
  html += ".error {background: #f8d7da; color: #721c24;}";
  html += "</style></head><body>";
  
  html += "<div class='container'>";
  html += "<h1>🤖 Zero Robot</h1>";
  html += "<p>Connect to your WiFi network</p>";
  
  if (wifiRetries > 0) {
    html += "<div style='background: #fff3cd; padding: 10px; border-radius: 5px; margin-bottom: 15px;'>";
    html += "<strong>Previous connection failed!</strong> Retry " + String(wifiRetries) + "/" + String(maxWifiRetries);
    html += "</div>";
  }
  
  html += "<h3>Available Networks:</h3>";
  html += "<div class='networks'>" + networks + "</div>";
  
  html += "<form id='wifiForm'>";
  html += "<input type='text' id='ssid' name='ssid' placeholder='WiFi Network Name' required>";
  html += "<input type='password' id='password' name='password' placeholder='WiFi Password'>";
  html += "<button type='button' class='btn btn-test' onclick='testConnection()'>Test Connection</button>";
  html += "<button type='submit' class='btn btn-primary'>Save & Connect</button>";
  html += "</form>";
  
  html += "<div id='status'></div>";
  html += "</div>";
  
  html += "<script>";
  html += "function selectNetwork(ssid) { document.getElementById('ssid').value = ssid; }";
  html += "function showStatus(message, type) {";
  html += "  var status = document.getElementById('status');";
  html += "  status.innerHTML = message;";
  html += "  status.className = type;";
  html += "  status.style.display = 'block';";
  html += "}";
  html += "function testConnection() {";
  html += "  var ssid = document.getElementById('ssid').value;";
  html += "  var password = document.getElementById('password').value;"; 
  html += "  if (!ssid) { showStatus('Please enter network name', 'error'); return; }";
  html += "  showStatus('Testing connection...', 'success');";
  html += "  fetch('/test', {";
  html += "    method: 'POST',";
  html += "    headers: {'Content-Type': 'application/x-www-form-urlencoded'},";
  html += "    body: 'ssid=/' + encodeURIComponent(ssid) + '/&password=/' + encodeURIComponent(password) + '/'";
  html += "  }).then(response => response.text()).then(data => {";
  html += "    showStatus(data, data.includes('Success') ? 'success' : 'error');";
  html += "  });";
  html += "}";
  html += "document.getElementById('wifiForm').addEventListener('submit', function(e) {";
  html += "  e.preventDefault();";
  html += "  var ssid = document.getElementById('ssid').value;";
  html += "  var password = document.getElementById('password').value;";
  html += "  if (!ssid) { showStatus('Please enter network name', 'error'); return; }";
  html += "  showStatus('Saving and connecting...', 'success');";
  html += "  fetch('/save', {";
  html += "    method: 'POST',";
  html += "    headers: {'Content-Type': 'application/x-www-form-urlencoded'},";
  html += "    body: 'ssid=/' + encodeURIComponent(ssid) + '/&password=/' + encodeURIComponent(password) + '/'";
  html += "  }).then(() => {";
  html += "    showStatus('Settings saved! Robot restarting...', 'success');";
  html += "  });";
  html += "});";
  html += "</script>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void handleTestCredentials() {
  String ssid = sanitizeInput(server.arg("ssid"));
  String password = sanitizeInput(server.arg("password"));
  
  Serial.println("Testing credentials: " + ssid);
  
  bool isValid = testWiFiConnection(ssid, password);
  
  if (isValid) {
    server.send(200, "text/plain", "Success: Connection test passed! Ready to save.");
  } else {
    server.send(200, "text/plain", "Error: Cannot connect to network. Check password and try again.");
  }
}

void handleSaveConfig() {
  String ssid = sanitizeInput(server.arg("ssid"));
  String password = sanitizeInput(server.arg("password"));
  
  if (ssid.length() == 0) {
    server.send(400, "text/plain", "Error: Network name cannot be empty");
    return;
  }
  
  Serial.println("Saving credentials: " + ssid);
  
  ssid_stored = ssid;
  password_stored = password;
  
  saveCredentials();
  
  server.send(200, "text/plain", "Settings saved successfully! Robot restarting...");
  
  delay(1000);
  ESP.restart();
}

String sanitizeInput(String input) {
  // **FIX FOR WEB INPUT ISSUES - REMOVE SLASHES WRAPPER**
  input.trim();
  
  // Remove our slash wrappers
  if (input.startsWith("/") && input.endsWith("/") && input.length() > 2) {
    input = input.substring(1, input.length() - 1);
  }
  
  // Remove problematic characters
  input.replace("\"", "");
  input.replace("'", "");
  input.replace("<", "");
  input.replace(">", "");
  input.replace("&", "");
  input.replace("\n", "");
  input.replace("\r", "");
  
  return input;
}

void saveCredentials() {
  writeStringToEEPROM(0, ssid_stored);
  writeStringToEEPROM(50, password_stored);
  EEPROM.commit();
  Serial.println("WiFi credentials saved to EEPROM");
}

void loadCredentials() {
  ssid_stored = readStringFromEEPROM(0);
  password_stored = readStringFromEEPROM(50);
  if (ssid_stored.length() > 0) {
    Serial.println("Loaded WiFi from EEPROM: " + ssid_stored);
  }
}

void clearCredentials() {
  for (int i = 0; i < 100; i++) { // Clear first 100 bytes
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  ssid_stored = "";
  password_stored = "";
  Serial.println("WiFi credentials cleared");
}

void writeStringToEEPROM(int addr, String data) {
  EEPROM.write(addr, data.length());
  for (unsigned int i = 0; i < data.length(); i++) {
    EEPROM.write(addr + 1 + i, data[i]);
  }
}

String readStringFromEEPROM(int addr) {
  int len = EEPROM.read(addr);
  String data = "";
  if (len > 0 && len < 30) { // Sanity check
    for (int i = 0; i < len; i++) {
      data += (char)EEPROM.read(addr + 1 + i);
    }
  }
  return data;
}

// Robot control functions
void carCommandUpdate() {
  if (word_command == "F") car_dir = "F";
  else if (word_command == "B") car_dir = "B";
  else if (word_command == "L") car_dir = "L";
  else if (word_command == "R") car_dir = "R";
  else if (word_command == "S") car_dir = "S";
}

void servoCommandUpdate() {
  int step = 3;
  
  // **FIXED: Servo control for 5-servo setup**
  if (word_command == "1") {          // Wrist CCW
    A2 = constrain(A2 - step, 0, 180);
  } else if (word_command == "4") {   // Wrist CW
    A2 = constrain(A2 + step, 0, 180);
  } else if (word_command == "2") {   // Claw UP
    A3 = constrain(A3 + step, 0, 180);
  } else if (word_command == "3") {   // Claw DOWN
    A3 = constrain(A3 - step, 0, 180);
  } else if (word_command == "U") {   // Full arm UP (dual motors mirrored)
    A4 = constrain(A4 + step, 0, 180);
    A5 = constrain(A5 - step, 0, 180);  // Mirrored movement
  } else if (word_command == "D") {   // Full arm DOWN (dual motors mirrored)
    A4 = constrain(A4 - step, 0, 180);
    A5 = constrain(A5 + step, 0, 180);  // Mirrored movement
  } else if (word_command == "C") {   // Claw close
    A1 = constrain(A1 + step, 0, 180);
  } else if (word_command == "O") {   // Claw open
    A1 = constrain(A1 - step, 0, 180);
  }
  
  delay(50);
}

void variableValueUpdate() {
  carCommandUpdate();
  servoCommandUpdate();  
}

void sendCommandToArduino() {
  // **FIXED: Send only 5 servo values to match your Arduino setup**
  String Command = "<" + car_dir + "," + car_speed + "," + 
                  "j1," + String(A1) + "," +   // Claw
                  "j2," + String(A2) + "," +   // Wrist rotation
                  "j3," + String(A3) + "," +   // Claw up/down
                  "j4," + String(A4) + "," +   // Full arm motor 1
                  "j5," + String(A5) + "," +   // Full arm motor 2 (mirrored)
                  "j6,90,j7,90>";              // Dummy values for compatibility
  Serial.println(Command);
}
