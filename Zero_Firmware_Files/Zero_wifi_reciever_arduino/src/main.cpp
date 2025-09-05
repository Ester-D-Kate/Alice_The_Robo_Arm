#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Define servos for 4-joint arm + base motors
Servo joint1Servo; // Claw (open/close)
Servo joint2Servo; // Wrist rotation
Servo joint3Servo; // Claw up/down joint
Servo joint4Servo; // Full arm motor 1
Servo joint5Servo; // Full arm motor 2 (mirrored)

Adafruit_MPU6050 mpu;

// Servo pins
const int joint1Pin = 17;    // Claw
const int joint2Pin = 16;     // Wrist rotation
const int joint3Pin = 15;    // Claw up/down
const int joint4Pin = 14;    // Full arm motor 1
const int joint5Pin = 4;    // Full arm motor 2 (mirrored)

// Motor control pins for H-bridge
const int motorLeftPin1 = 3;
const int motorLeftPin2 = 5;
const int motorRightPin1 = 6;
const int motorRightPin2 = 11;

// Joint positions
int joint1Pos = 90;  // Claw position
int joint2Pos = 90;  // Wrist position
int joint3Pos = 90;  // Claw vertical position
int joint4Pos = 90;  // Full arm motor 1
int joint5Pos = 90;  // Full arm motor 2 (mirrored)

String carDirection = "S";
int carSpeed = 0;

// **IMPROVED PID CONSTANTS**
float Kp = 5.0;   
float Ki = 0.000; 
float Kd = 0.0;   
float desired_angle = 0;
float integral = 0.0;
float previous_error = 0.0;
float angleZ = 0.0;
float gyroZOffset = 0.0;
unsigned long prevTime = 0;
int DegreeAngle = 0;

// **GYRO DRIFT FIX**
int stationaryCount = 0;
const float GYRO_DEAD_ZONE = 0.02;
unsigned long lastDriftCorrection = 0;

// Battery and buzzer
const int buzzerPin = 8;
const float lowVoltageThreshold = 3.5;
bool lowBatteryWarning = false;
unsigned long lastBuzzerTime = 0;
bool buzzerState = false;

// PID output limits
const float PID_MIN = -100.0;
const float PID_MAX = 100.0;
const float MOTOR_PWM_MIN = 0;
const float MOTOR_PWM_MAX = 255;

bool isFDesiredAngleSet = false;
bool isBDesiredAngleSet = false;
bool isLDesiredAngleSet = false;
bool isRDesiredAngleSet = false;
bool isSDesiredAngleSet = false;

String receivedCommand = "";
boolean commandComplete = false;

void resetPID() {
    integral = 0;
    previous_error = 0;
}

float computePID(float setpoint, float current_value, float dt) {
    if (dt <= 0 || dt > 0.1) dt = 0.01; 
    
    float error = setpoint - current_value;
    integral += error * dt; 
    integral = constrain(integral, -50.0, 50.0);
    
    float derivative = (error - previous_error) / dt;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    output = constrain(output, PID_MIN, PID_MAX);
    
    previous_error = error;
    return output;
}

void setup() {
  Serial.begin(57600);
  
  Serial.println("=== ALICE ROBOT STARTING ===");
  
  pinMode(A6, INPUT);
  
  // Setup motor control pins
  pinMode(motorLeftPin1, OUTPUT);
  pinMode(motorLeftPin2, OUTPUT);
  pinMode(motorRightPin1, OUTPUT);
  pinMode(motorRightPin2, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  
  // Initialize motors to stopped state
  analogWrite(motorLeftPin1, 0);
  analogWrite(motorLeftPin2, 0);
  analogWrite(motorRightPin1, 0);
  analogWrite(motorRightPin2, 0);
  
  Serial.println("Motors initialized");
  
  // **ATTACH SERVOS - NO DELAYS**
  joint1Servo.attach(joint1Pin);
  joint2Servo.attach(joint2Pin);
  joint3Servo.attach(joint3Pin);
  joint4Servo.attach(joint4Pin);
  joint5Servo.attach(joint5Pin);
  
  // Initialize servos to middle positions
  joint1Servo.write(joint1Pos);
  joint2Servo.write(joint2Pos);
  joint3Servo.write(joint3Pos);
  joint4Servo.write(joint4Pos);
  joint5Servo.write(joint5Pos);
  
  Serial.println("Servos attached and positioned");

  // **SIMPLE MPU6050 INIT**
  Serial.println("Initializing MPU6050...");
  Wire.begin();
  
  if (mpu.begin()) {
    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    // **QUICK GYRO CALIBRATION - NO DELAYS**
    Serial.println("Quick gyro calibration...");
    float sum = 0;
    int validSamples = 0;
    
    for (int i = 0; i < 50; i++) {  // Reduced from 100 to 50
      sensors_event_t a, g, temp;
      if (mpu.getEvent(&a, &g, &temp)) {
        sum += g.gyro.z;
        validSamples++;
      }
    }
    
    if (validSamples > 0) {
      gyroZOffset = sum / validSamples;
      Serial.print("Gyro offset: ");
      Serial.println(gyroZOffset, 4);
    }
  } else {
    Serial.println("MPU6050 failed - continuing without gyro");
  }

  prevTime = millis();
  lastDriftCorrection = millis();
  
  Serial.println("=== ALICE ROBOT READY ===");
  Serial.println("Send: <F,50,j1,90,j2,90,j3,90,j4,90,j5,90>");
  Serial.println("Status updates every 3 seconds...");
}

void controlCar(String direction, int speed, float pidCorrection = 0) {
  int pwmSpeed = map(speed, 0, 100, 0, 255);
  int leftSpeed, rightSpeed;
  
  pidCorrection = constrain(pidCorrection, -254, 254);
  
  if (direction == "F" || direction == "FWD" || direction == "FORWARD") {
    leftSpeed = constrain(pwmSpeed + pidCorrection, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    rightSpeed = constrain(pwmSpeed - pidCorrection, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    
    analogWrite(motorLeftPin1, leftSpeed);
    analogWrite(motorLeftPin2, 0);
    analogWrite(motorRightPin1, rightSpeed);
    analogWrite(motorRightPin2, 0);
  } 
  else if (direction == "B" || direction == "BWD" || direction == "BACKWARD") {
    leftSpeed = constrain(pwmSpeed - pidCorrection, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    rightSpeed = constrain(pwmSpeed + pidCorrection, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    
    analogWrite(motorLeftPin1, 0);
    analogWrite(motorLeftPin2, leftSpeed);
    analogWrite(motorRightPin1, 0);
    analogWrite(motorRightPin2, rightSpeed);
  }
  else if (direction == "R" || direction == "RIGHT"){
    analogWrite(motorLeftPin1, 0);
    analogWrite(motorLeftPin2, pwmSpeed);
    analogWrite(motorRightPin1, pwmSpeed);
    analogWrite(motorRightPin2, 0);
  }
  else if (direction == "L" || direction == "LEFT") {
    analogWrite(motorLeftPin1, pwmSpeed);
    analogWrite(motorLeftPin2, 0);
    analogWrite(motorRightPin1, 0);
    analogWrite(motorRightPin2, pwmSpeed);
  }
  else {
    analogWrite(motorLeftPin1, 0);
    analogWrite(motorLeftPin2, 0);
    analogWrite(motorRightPin1, 0);
    analogWrite(motorRightPin2, 0);
  }
}

void processCommand(String command) {
  Serial.print("Processing: ");
  Serial.println(command);
  
  command.trim();
  
  if (command.length() < 5) {
    Serial.println("Command too short");
    return;
  }
  
  // **AUTO-FIX MISSING BRACKETS**
  if (!command.startsWith("<")) {
    command = "<" + command;
  }
  if (!command.endsWith(">")) {
    command = command + ">";
  }
  
  // Remove brackets
  command = command.substring(1, command.length() - 1);
  
  // **SPLIT BY COMMAS**
  String parts[20];
  int partCount = 0;
  int lastIndex = 0;
  
  for (int i = 0; i <= command.length(); i++) {
    if (i == command.length() || command[i] == ',') {
      if (partCount < 20) {
        parts[partCount] = command.substring(lastIndex, i);
        parts[partCount].trim();
        partCount++;
      }
      lastIndex = i + 1;
    }
  }
  
  if (partCount < 2) {
    Serial.println("Not enough parts");
    return;
  }
  
  // **PARSE CAR DIRECTION AND SPEED**
  carDirection = parts[0];
  carSpeed = parts[1].toInt();
  carSpeed = constrain(carSpeed, 0, 100);
  
  Serial.print("Car: ");
  Serial.print(carDirection);
  Serial.print(", Speed: ");
  Serial.println(carSpeed);
  
  // **PARSE SERVOS**
  for (int i = 2; i < partCount - 1; i += 2) {
    String joint = parts[i];
    int pos = parts[i + 1].toInt();
    pos = constrain(pos, 0, 180);
    
    if (joint == "j1") {
      joint1Pos = pos;
      joint1Servo.write(joint1Pos);
      Serial.print("J1: ");
      Serial.println(joint1Pos);
    } else if (joint == "j2") {
      joint2Pos = pos;
      joint2Servo.write(joint2Pos);
      Serial.print("J2: ");
      Serial.println(joint2Pos);
    } else if (joint == "j3") {
      joint3Pos = pos;
      joint3Servo.write(joint3Pos);
      Serial.print("J3: ");
      Serial.println(joint3Pos);
    } else if (joint == "j4") {
      joint4Pos = pos;
      joint4Servo.write(joint4Pos);
      Serial.print("J4: ");
      Serial.println(joint4Pos);
    } else if (joint == "j5") {
      joint5Pos = pos;
      joint5Servo.write(joint5Pos);
      Serial.print("J5: ");
      Serial.println(joint5Pos);
    }
  }
  
  Serial.println("Command processed successfully!");
}

void checkLowBattery(float voltage) {
  if (voltage < lowVoltageThreshold && voltage > 1.5) {
    if (!lowBatteryWarning) {
      lowBatteryWarning = true;
      Serial.println("WARNING: Low Battery!");
    }
    
    unsigned long currentTime = millis();
    if (currentTime - lastBuzzerTime >= 500) {
      buzzerState = !buzzerState;
      digitalWrite(buzzerPin, buzzerState ? HIGH : LOW);
      lastBuzzerTime = currentTime;
    }
  } else {
    if (lowBatteryWarning) {
      lowBatteryWarning = false;
      digitalWrite(buzzerPin, LOW);
      buzzerState = false;
    }
  }
}

void loop() {
  int analogValue = analogRead(A6);
  float voltage = analogValue * (5.0 / 1023.0);
  
  checkLowBattery(voltage);

  // **GYRO PROCESSING**
  sensors_event_t a, g, temp;
  bool gyroWorking = mpu.getEvent(&a, &g, &temp);
  
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  
  if (dt > 0.1) dt = 0.01;
  if (dt < 0.001) dt = 0.001;
  
  prevTime = currentTime;
  
  float gyroZ = 0;
  if (gyroWorking) {
    gyroZ = g.gyro.z - gyroZOffset;
    
    if (abs(gyroZ) < GYRO_DEAD_ZONE) {
      gyroZ = 0;
      stationaryCount++;
    } else {
      stationaryCount = 0;
    }
    
    angleZ += gyroZ * dt;
    
    if (stationaryCount > 1000 && millis() - lastDriftCorrection > 5000) {
      if (abs(angleZ) < 0.05) {
        angleZ = 0;
      } else {
        angleZ *= 0.99;
      }
      lastDriftCorrection = millis();
    }
    
    if (angleZ > 6.19) {
      angleZ = 0;
    } else if (angleZ < -6.20) {
      angleZ = 0;
    }
  }
  
  DegreeAngle = (angleZ / 6.20) * 360;
  
  int RotationalAngle = DegreeAngle;
  if (DegreeAngle > 179) {
    RotationalAngle = DegreeAngle - 360;
  } else if (DegreeAngle < -179) {
    RotationalAngle = DegreeAngle + 360;
  }
  
  // **SERIAL READING**
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    receivedCommand += inChar;
    
    if (inChar == '\n' || inChar == '\r' || inChar == '>') {
      commandComplete = true;
    }
    
    if (receivedCommand.length() > 200) {
      receivedCommand = "";
      commandComplete = false;
    }
  }
  
  if (commandComplete) {
    processCommand(receivedCommand);
    receivedCommand = "";
    commandComplete = false;
    
    isFDesiredAngleSet = false;
    isBDesiredAngleSet = false;
    isLDesiredAngleSet = false;
    isRDesiredAngleSet = false;
    isSDesiredAngleSet = false;
  }
  
  // **PID CONTROL**
  float pidOutput = 0;
  
  if (carDirection == "F") {
    if (!isFDesiredAngleSet) { 
      desired_angle = RotationalAngle;
      resetPID();
      isFDesiredAngleSet = true;
    }
    if (gyroWorking) {
      pidOutput = computePID(desired_angle, RotationalAngle, dt);
    }
    controlCar(carDirection, carSpeed, pidOutput);
  } 
  else if (carDirection == "B") {
    if (!isBDesiredAngleSet) { 
      desired_angle = RotationalAngle;
      resetPID();
      isBDesiredAngleSet = true;
    }
    if (gyroWorking) {
      pidOutput = computePID(desired_angle, RotationalAngle, dt);
    }
    controlCar(carDirection, carSpeed, pidOutput);
  }
  else {
    controlCar(carDirection, carSpeed);
  }
  
  // **REGULAR STATUS OUTPUT**
  static unsigned long lastDebugTime = 0;
  if (currentTime - lastDebugTime > 3000) {
    lastDebugTime = currentTime;
    Serial.print("Status - Angle: ");
    Serial.print(RotationalAngle);
    Serial.print(", Car: ");
    Serial.print(carDirection);
    Serial.print(", Speed: ");
    Serial.print(carSpeed);
    Serial.print(", Battery: ");
    Serial.print(voltage, 1);
    Serial.print("V, Uptime: ");
    Serial.print(millis() / 1000);
    Serial.println("s");
  }
  
  delay(10);
}