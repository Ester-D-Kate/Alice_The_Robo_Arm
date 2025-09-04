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
Servo joint6Servo; // Extra joint (unused)
Servo joint7Servo; // Extra joint (unused)

Adafruit_MPU6050 mpu;

// Servo pins
const int joint1Pin = 14;    // Claw
const int joint2Pin = 4;     // Wrist rotation
const int joint3Pin = 15;    // Claw up/down
const int joint4Pin = 16;    // Full arm motor 1
const int joint5Pin = 17;    // Full arm motor 2 (mirrored)


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
int joint6Pos = 90;  // Unused
int joint7Pos = 90;  // Unused

String carDirection = "S";
int carSpeed = 0;

// **IMPROVED PID CONSTANTS**
float Kp = 5.0;   // Reduced from 1.5 for stability
float Ki = 0.000; // Reduced from 0.01 for less aggressive integral action
float Kd = 0.0;   // Reduced from 0.5 for less derivative kick
float desired_angle = 0;
float integral = 0.0;
float previous_error = 0.0;
float angleZ = 0.0;
float gyroZOffset = 0.0;
unsigned long prevTime = 0;
int DegreeAngle = 0;

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
    // Prevent division by zero and ensure reasonable dt
    if (dt <= 0 || dt > 0.1) dt = 0.01; 
    
    float error = setpoint - current_value;
    integral += error * dt; 
    
    // **IMPROVED INTEGRAL WINDUP PROTECTION**
    integral = constrain(integral, -50.0, 50.0);
    
    float derivative = (error - previous_error) / dt;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    // **LIMIT PID OUTPUT TO PREVENT MOTOR OVERFLOW**
    output = constrain(output, PID_MIN, PID_MAX);
    
    previous_error = error;
    return output;
}

void setup() {
  Serial.begin(57600);
  
  pinMode(A6, INPUT);
  Wire.begin();
  
  // Setup motor control pins
  pinMode(motorLeftPin1, OUTPUT);
  pinMode(motorLeftPin2, OUTPUT);
  pinMode(motorRightPin1, OUTPUT);
  pinMode(motorRightPin2, OUTPUT);
  
  // Initialize motors to stopped state
  analogWrite(motorLeftPin1, 0);
  analogWrite(motorLeftPin2, 0);
  analogWrite(motorRightPin1, 0);
  analogWrite(motorRightPin2, 0);
  
  // **ATTACH SERVOS FOR 4-JOINT ARM**
  joint1Servo.attach(joint1Pin);  // Claw
  joint2Servo.attach(joint2Pin);  // Wrist rotation
  joint3Servo.attach(joint3Pin);  // Claw up/down
  joint4Servo.attach(joint4Pin);  // Full arm motor 1
  joint5Servo.attach(joint5Pin);  // Full arm motor 2 (mirrored)
  
  // Initialize servos to middle positions
  joint1Servo.write(joint1Pos);
  joint2Servo.write(joint2Pos);
  joint3Servo.write(joint3Pos);
  joint4Servo.write(joint4Pos);
  joint5Servo.write(joint5Pos);

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Calculate Gyro Drift Offset
  Serial.println("Calculating Gyro Drift...");
  int numSamples = 500;
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroZOffset += g.gyro.z;
    delay(5);
  }
  gyroZOffset /= numSamples;
  Serial.print("Gyro Z Offset: ");
  Serial.println(gyroZOffset, 6);

  prevTime = millis();
  Serial.println("4-Joint Arm Robot initialized and ready!");
}

void controlCar(String direction, int speed, float pidCorrection = 0) {
  int pwmSpeed = map(speed, 0, 100, 0, 255);
  int leftSpeed, rightSpeed;
  
  // **CONSTRAIN PID CORRECTION TO MOTOR PWM RANGE**
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
  else if (direction == "S" || direction == "STOP") {
    analogWrite(motorLeftPin1, 0);
    analogWrite(motorLeftPin2, 0);
    analogWrite(motorRightPin1, 0);
    analogWrite(motorRightPin2, 0);
  }
}

void processCommand(String command) {
  command.trim();
  
  if (!command.startsWith("<") || !command.endsWith(">") || command.length() < 3) {
    Serial.println("Invalid command format");
    return;
  }
  
  command = command.substring(1, command.length() - 1);
  
  if (command.indexOf(',') == -1 || command.indexOf(',', command.indexOf(',') + 1) == -1) {
    Serial.println("Invalid command format - missing required components");
    return;
  }
  
  // Parse the comma-separated values
  int commaIndex = 0;
  int nextCommaIndex = 0;
  
  // Get car direction
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  String dirStr = command.substring(commaIndex, nextCommaIndex);
  if (dirStr.length() > 0) {
    carDirection = dirStr;
  }
  commaIndex = nextCommaIndex + 1;
  
  // Get car speed
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  int speedVal = command.substring(commaIndex, nextCommaIndex).toInt();
  if (speedVal >= 0 && speedVal <= 100) {
    carSpeed = speedVal;
  }
  commaIndex = nextCommaIndex + 1;
  
  if (command.indexOf("j1,") == -1) {
    Serial.print("Partial command - Car: ");
    Serial.print(carDirection);
    Serial.print(",");
    Serial.println(carSpeed);
    return;
  }
  
  // **PARSE ALL 7 JOINTS (MAINTAINING COMPATIBILITY)**
  // Joint 1 (Claw)
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  commaIndex = nextCommaIndex + 1;
  
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  joint1Pos = constrain(command.substring(commaIndex, nextCommaIndex).toInt(), 0, 180);
  joint1Servo.write(joint1Pos);
  commaIndex = nextCommaIndex + 1;
  
  // Joint 2 (Wrist rotation)
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  commaIndex = nextCommaIndex + 1;
  
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  joint2Pos = constrain(command.substring(commaIndex, nextCommaIndex).toInt(), 0, 180);
  joint2Servo.write(joint2Pos);
  commaIndex = nextCommaIndex + 1;
  
  // Joint 3 (Claw up/down)
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  commaIndex = nextCommaIndex + 1;
  
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  joint3Pos = constrain(command.substring(commaIndex, nextCommaIndex).toInt(), 0, 180);
  joint3Servo.write(joint3Pos);
  commaIndex = nextCommaIndex + 1;
  
  // Joint 4 (Full arm motor 1)
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  commaIndex = nextCommaIndex + 1;
  
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  joint4Pos = constrain(command.substring(commaIndex, nextCommaIndex).toInt(), 0, 180);
  joint4Servo.write(joint4Pos);
  commaIndex = nextCommaIndex + 1;
  
  // Joint 5 (Full arm motor 2 - mirrored)
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  commaIndex = nextCommaIndex + 1;
  
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  joint5Pos = constrain(command.substring(commaIndex, nextCommaIndex).toInt(), 0, 180);
  joint5Servo.write(joint5Pos);
  commaIndex = nextCommaIndex + 1;
  
  // Joints 6 & 7 (parse but don't use for 4-joint arm)
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  commaIndex = nextCommaIndex + 1;
  
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  joint6Pos = constrain(command.substring(commaIndex, nextCommaIndex).toInt(), 0, 180);
  commaIndex = nextCommaIndex + 1;
  
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  commaIndex = nextCommaIndex + 1;
  
  joint7Pos = constrain(command.substring(commaIndex).toInt(), 0, 180);
  
  // **DEBUG OUTPUT FOR 4-JOINT ARM**
  Serial.print("4-Joint Arm - Car: ");
  Serial.print(carDirection);
  Serial.print(",");
  Serial.print(carSpeed);
  Serial.print(" | Claw:");
  Serial.print(joint1Pos);
  Serial.print(" Wrist:");
  Serial.print(joint2Pos);
  Serial.print(" ClawUD:");
  Serial.print(joint3Pos);
  Serial.print(" ArmM1:");
  Serial.print(joint4Pos);
  Serial.print(" ArmM2:");
  Serial.println(joint5Pos);
}

void loop() {
  int analogValue = analogRead(A6);
  float voltage = analogValue * (5.0 / 1023.0);

  // **IMPROVED GYRO PROCESSING**
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  
  // **PREVENT dt FROM BEING TOO LARGE OR SMALL**
  if (dt > 0.1) dt = 0.01; // Cap at 100ms
  if (dt < 0.001) dt = 0.001; // Min 1ms
  
  prevTime = currentTime;
  
  float gyroZ = g.gyro.z - gyroZOffset;  
  angleZ += gyroZ * dt;
  
  // **KEEP YOUR 6.20 CONSTANT (DON'T CHANGE)**
  if (angleZ > 6.19) {
    angleZ = 0;
  } else if (angleZ < -6.20) {
    angleZ = 0;
  }
  
  DegreeAngle = (angleZ / 6.20) * 360;
  
  int RotationalAngle = DegreeAngle;
  if (DegreeAngle > 179) {
    RotationalAngle = DegreeAngle - 360;
  } else if (DegreeAngle < -179) {
    RotationalAngle = DegreeAngle + 360;
  }
  
  // Read from Serial port
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    receivedCommand += inChar;
    if (inChar == '\n' || inChar == '>') {
      commandComplete = true;
    }
  }
  
  if (commandComplete) {
    String prevDirection = carDirection;
    processCommand(receivedCommand);
    receivedCommand = "";
    commandComplete = false;
    
    if (prevDirection != carDirection) {
      isFDesiredAngleSet = false;
      isBDesiredAngleSet = false;
      isLDesiredAngleSet = false;
      isRDesiredAngleSet = false;
      isSDesiredAngleSet = false;
    }
  }
  
  // **IMPROVED PID CONTROL APPLICATION**
  float pidOutput = 0;
  
  if (carDirection == "F" || carDirection == "FWD" || carDirection == "FORWARD") {
    if (!isFDesiredAngleSet) { 
      desired_angle = RotationalAngle;
      resetPID();
      isFDesiredAngleSet = true;
      Serial.print("Forward - Locked angle: ");
      Serial.println(desired_angle);
    }
    pidOutput = computePID(desired_angle, RotationalAngle, dt);
    controlCar(carDirection, carSpeed, pidOutput);
  } 
  else if (carDirection == "B" || carDirection == "BWD" || carDirection == "BACKWARD") {
    if (!isBDesiredAngleSet) { 
      desired_angle = RotationalAngle;
      resetPID();
      isBDesiredAngleSet = true;
      Serial.print("Backward - Locked angle: ");
      Serial.println(desired_angle);
    }
    pidOutput = computePID(desired_angle, RotationalAngle, dt);
    controlCar(carDirection, carSpeed, pidOutput);
  }
  else {
    controlCar(carDirection, carSpeed);
    
    if (carDirection == "L") {
      if (!isLDesiredAngleSet) {
        isLDesiredAngleSet = true;
      }
    } else {
      isLDesiredAngleSet = false;
    }
    
    if (carDirection == "R") {
      if (!isRDesiredAngleSet) {
        isRDesiredAngleSet = true;
      }
    } else {
      isRDesiredAngleSet = false;
    }
    
    if (carDirection == "S") {
      if (!isSDesiredAngleSet) {
        isSDesiredAngleSet = true;
      }
    } else {
      isSDesiredAngleSet = false;
    }
  }
  
  if (carDirection != "F" && carDirection != "FWD" && carDirection != "FORWARD") {
    isFDesiredAngleSet = false;
  }
  
  if (carDirection != "B" && carDirection != "BWD" && carDirection != "BACKWARD") {
    isBDesiredAngleSet = false;
  }
  
  // **REDUCED DEBUG OUTPUT FREQUENCY**
  static unsigned long lastDebugTime = 0;
  if (currentTime - lastDebugTime > 1000) { // Every 1 second instead of 500ms
    lastDebugTime = currentTime;
    Serial.print("Angle: ");
    Serial.print(RotationalAngle);
    Serial.print(", Desired: ");
    Serial.print(desired_angle);
    if (carDirection == "F" || carDirection == "B") {
      Serial.print(", PID: ");
      Serial.println(pidOutput);
    } else {
      Serial.println();
    }
  }
  
  delay(5);
}