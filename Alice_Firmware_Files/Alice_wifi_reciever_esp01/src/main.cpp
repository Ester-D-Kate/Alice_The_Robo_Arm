#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// MQTT Configuration
const char* mqtt_server = "broker.hivemq.com";
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
const char* arm_position_topic = "alice/zero/arm/position";  // **NEW: For absolute positioning**
const char* status_topic = "alice/zero/robot/status";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
ESP8266WebServer server(80);

unsigned long lastStatusTime = 0;
unsigned long commandTimeout = 0;
unsigned long lastMoveTime = 0;

void setup() {
  Serial.begin(57600);
  EEPROM.begin(512);
  
  Serial.println("========================================");
  Serial.println("    ZERO ROBOT ENHANCED CONTROL");
  Serial.println("========================================");
  
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
        Serial.println("Starting config mode...");
        startConfigMode();
      } else {
        connectToWiFi();
      }
    }
  }
  
  delay(20); // Reduced for smoother servo movement
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
      
      // **SUBSCRIBE TO ALL CONTROL TOPICS**
      mqttClient.subscribe(car_command_topic);
      mqttClient.subscribe(arm_command_topic);
      mqttClient.subscribe(arm_position_topic);  // **NEW: Position control**
      
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
  
  // Parse JSON commands
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    // Handle simple string commands for backward compatibility
    if (message.length() <= 2) {
      word_command = message;
      controlMode = MANUAL_CONTINUOUS;
      Serial.println("Manual command: " + word_command);
      return;
    }
  } else {
    // **ENHANCED COMMAND HANDLING**
    if (topicStr == car_command_topic) {
      handleCarCommand(doc);
    } else if (topicStr == arm_command_topic) {
      handleArmCommand(doc);
    } else if (topicStr == arm_position_topic) {
      handlePositionCommand(doc);  // **NEW: Handle absolute positioning**
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
    
    // **MANUAL CONTINUOUS MOVEMENT**
    if (cmd == "U" || cmd == "D" || cmd == "C" || cmd == "O" || 
        cmd == "1" || cmd == "2" || cmd == "3" || cmd == "4" || cmd == "S") {
      word_command = cmd;
      controlMode = MANUAL_CONTINUOUS;
      Serial.println("Manual arm command: " + cmd);
      
      // Set movement flags for servos
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

// **NEW: HANDLE ABSOLUTE POSITION COMMANDS**
void handlePositionCommand(JsonDocument& doc) {
  Serial.println("Position command received");
  controlMode = POSITION_ABSOLUTE;
  word_command = "S"; // Stop any manual movement
  
  // Set individual servo positions
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
  // Set movement flags based on command
  if (cmd == "U") {          // Full arm UP
    servos[3].isMoving = true;  // A4
    servos[4].isMoving = true;  // A5 (mirrored)
  } else if (cmd == "D") {   // Full arm DOWN
    servos[3].isMoving = true;  // A4
    servos[4].isMoving = true;  // A5 (mirrored)
  } else if (cmd == "C") {   // Claw close
    servos[0].isMoving = true;  // A1
  } else if (cmd == "O") {   // Claw open
    servos[0].isMoving = true;  // A1
  } else if (cmd == "1") {   // Wrist CCW
    servos[1].isMoving = true;  // A2
  } else if (cmd == "4") {   // Wrist CW
    servos[1].isMoving = true;  // A2
  } else if (cmd == "2") {   // Claw UP
    servos[2].isMoving = true;  // A3
  } else if (cmd == "3") {   // Claw DOWN
    servos[2].isMoving = true;  // A3
  }
}

void stopAllMovement() {
  for (int i = 0; i < 5; i++) {
    servos[i].isMoving = false;
  }
  Serial.println("All servo movement stopped");
}

// **ENHANCED ROBOT CONTROL UPDATE**
void updateRobotControl() {
  carCommandUpdate();
  
  if (millis() - lastMoveTime > 50) { // Move every 50ms for smooth operation
    if (controlMode == MANUAL_CONTINUOUS) {
      updateManualMovement();
    } else if (controlMode == POSITION_ABSOLUTE) {
      updatePositionMovement();
    }
    lastMoveTime = millis();
  }
}

void updateManualMovement() {
  int step = 2; // Smaller steps for smoother movement
  
  if (word_command == "1" && servos[1].isMoving) {          // Wrist CCW
    servos[1].currentPosition = constrain(servos[1].currentPosition - step, 0, 180);
  } else if (word_command == "4" && servos[1].isMoving) {   // Wrist CW
    servos[1].currentPosition = constrain(servos[1].currentPosition + step, 0, 180);
  } else if (word_command == "2" && servos[2].isMoving) {   // Claw UP
    servos[2].currentPosition = constrain(servos[2].currentPosition + step, 0, 180);
  } else if (word_command == "3" && servos[2].isMoving) {   // Claw DOWN
    servos[2].currentPosition = constrain(servos[2].currentPosition - step, 0, 180);
  } else if (word_command == "U") {   // Full arm UP
    if (servos[3].isMoving) servos[3].currentPosition = constrain(servos[3].currentPosition + step, 0, 180);
    if (servos[4].isMoving) servos[4].currentPosition = constrain(servos[4].currentPosition - step, 0, 180);  // Mirrored
  } else if (word_command == "D") {   // Full arm DOWN
    if (servos[3].isMoving) servos[3].currentPosition = constrain(servos[3].currentPosition - step, 0, 180);
    if (servos[4].isMoving) servos[4].currentPosition = constrain(servos[4].currentPosition + step, 0, 180);  // Mirrored
  } else if (word_command == "C" && servos[0].isMoving) {   // Claw close
    servos[0].currentPosition = constrain(servos[0].currentPosition + step, 0, 180);
  } else if (word_command == "O" && servos[0].isMoving) {   // Claw open
    servos[0].currentPosition = constrain(servos[0].currentPosition - step, 0, 180);
  }
}

void updatePositionMovement() {
  int step = 3; // Step size for position movement
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
        servos[i].isMoving = false; // Reached target
      }
    }
  }
  
  // Switch back to manual mode when all movements complete
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
  
  // **REAL-TIME SERVO POSITIONS**
  JsonObject positions = arm["positions"].to<JsonObject>();
  positions["A1"] = servos[0].currentPosition;  // Claw
  positions["A2"] = servos[1].currentPosition;  // Wrist rotation
  positions["A3"] = servos[2].currentPosition;  // Claw up/down
  positions["A4"] = servos[3].currentPosition;  // Full arm motor 1
  positions["A5"] = servos[4].currentPosition;  // Full arm motor 2
  
  // **MOVEMENT STATUS**
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

void sendCommandToArduino() {
  // Send current servo positions to Arduino
  String Command = "<" + car_dir + "," + car_speed + "," + 
                  "j1," + String(servos[0].currentPosition) + "," +   // Claw
                  "j2," + String(servos[1].currentPosition) + "," +   // Wrist rotation
                  "j3," + String(servos[2].currentPosition) + "," +   // Claw up/down
                  "j4," + String(servos[3].currentPosition) + "," +   // Full arm motor 1
                  "j5," + String(servos[4].currentPosition) + "," +   // Full arm motor 2
                  "j6,90,j7,90>";                                     // Dummy values
  Serial.println(Command);
}

// [Previous WiFi and configuration functions remain the same...]
// [Include all the WiFi setup, config mode, EEPROM functions from your original code]
