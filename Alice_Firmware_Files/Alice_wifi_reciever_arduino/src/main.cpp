#include <Arduino.h>  // Include the Arduino core library
#include <Servo.h>  // Include the Servo library
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Define servos for all joints
Servo joint1Servo; // Joint 1
Servo joint2Servo; // Joint 2
Servo joint3Servo; // Joint 3
Servo joint4Servo; // Joint 4
Servo joint5Servo; // Joint 5
Servo joint6Servo; // Joint 6
Servo joint7Servo; // Joint 7

// MPU6050 gyroscope
Adafruit_MPU6050 mpu;

// Define pins for all servos
const int joint1Pin = 12;    // Pin for joint 1
const int joint2Pin = 13;    // Pin for joint 2
const int joint3Pin = 14;    // Pin for joint 3
const int joint4Pin = 15;    // Pin for joint 4
const int joint5Pin = 16;    // Pin for joint 5
const int joint6Pin = 17;    // Pin for joint 6
const int joint7Pin = 18;    // Pin for joint 7

// Motor control pins for H-bridge
const int motorLeftPin1 = 3;    // Left motor direction pin 1 (PWM for speed control)
const int motorLeftPin2 = 5;    // Left motor direction pin 2 (PWM for speed control)
const int motorRightPin1 = 6;   // Right motor direction pin 1 (PWM for speed control)
const int motorRightPin2 = 11;   // Right motor direction pin 2 (PWM for speed control)

// Joint positions
int joint1Pos = 90;
int joint2Pos = 90;
int joint3Pos = 90;
int joint4Pos = 90;
int joint5Pos = 90;
int joint6Pos = 90;
int joint7Pos = 90;

// Car control variables
String carDirection = "S";
int carSpeed = 0;

// PID control variables
float Kp = 1.5;  // Proportional Gain
float Ki = 0.01; // Integral Gain
float Kd = 0.5;  // Derivative Gain
float desired_angle = 0;  // Setpoint (desired angle)
float integral = 0.0;
float previous_error = 0.0;
float angleZ = 0.0;  // Total rotated angle
float gyroZOffset = 0.0;  // Will be calculated
unsigned long prevTime = 0;  // Previous time
int DegreeAngle = 0;

// Flags for PID control
bool isFDesiredAngleSet = false;
bool isBDesiredAngleSet = false;
bool isLDesiredAngleSet = false;
bool isRDesiredAngleSet = false;
bool isSDesiredAngleSet = false;

String receivedCommand = "";
boolean commandComplete = false;

// Function to reset PID values
void resetPID() {
    integral = 0;
    previous_error = 0;
}

float computePID(float setpoint, float current_value, float dt) {
    float error = setpoint - current_value;
    integral += error * dt; 
    
    // Limit integral windup
    if (integral > 100) integral = 100;
    if (integral < -100) integral = -100;
    
    float derivative = (error - previous_error) / dt;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    previous_error = error;
    
    return output;
}

void setup() {
  Serial.begin(57600);  // Start serial communication at the same baud rate as ESP8266
  
  // Initialize I2C for MPU6050
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
  
  // Attach all servos to their pins
  joint1Servo.attach(joint1Pin);
  joint2Servo.attach(joint2Pin);
  joint3Servo.attach(joint3Pin);
  joint4Servo.attach(joint4Pin);
  joint5Servo.attach(joint5Pin);
  joint6Servo.attach(joint6Pin);
  joint7Servo.attach(joint7Pin);
  
  // Initialize all servos to middle position
  joint1Servo.write(joint1Pos);
  joint2Servo.write(joint2Pos);
  joint3Servo.write(joint3Pos);
  joint4Servo.write(joint4Pos);
  joint5Servo.write(joint5Pos);
  joint6Servo.write(joint6Pos);
  joint7Servo.write(joint7Pos);

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
    delay(5);  // Small delay for stability
  }
  gyroZOffset /= numSamples;  // Average drift calculation
  Serial.print("Gyro Z Offset (Drift Correction): ");
  Serial.println(gyroZOffset, 6);

  prevTime = millis();  // Initialize time for PID control

  Serial.println("MPU6050 initialized");
  Serial.println("Arduino initialized and ready to receive commands from ESP");
}

// Function to control car movement with PWM for H-bridge and PID correction
void controlCar(String direction, int speed, float pidCorrection = 0) {
  // Map speed (0-100) to PWM value (0-255)
  int pwmSpeed = map(speed, 0, 100, 0, 255);
  int leftSpeed, rightSpeed;
  
  // Apply PID correction to motor speeds
  if (direction == "F" || direction == "FWD" || direction == "FORWARD") {
    // Move forward with PID correction
    leftSpeed = constrain(pwmSpeed - pidCorrection, 0, 255);
    rightSpeed = constrain(pwmSpeed + pidCorrection, 0, 255);
    
    analogWrite(motorLeftPin1, leftSpeed);    // PWM for left motor forward
    analogWrite(motorLeftPin2, 0);            // Left motor backward off
    analogWrite(motorRightPin1, rightSpeed);  // PWM for right motor forward
    analogWrite(motorRightPin2, 0);           // Right motor backward off
  } 
  else if (direction == "B" || direction == "BWD" || direction == "BACKWARD") {
    // Move backward with PID correction
    leftSpeed = constrain(pwmSpeed + pidCorrection, 0, 255);
    rightSpeed = constrain(pwmSpeed - pidCorrection, 0, 255);
    
    analogWrite(motorLeftPin1, 0);            // Left motor forward off
    analogWrite(motorLeftPin2, leftSpeed);    // PWM for left motor backward
    analogWrite(motorRightPin1, 0);           // Right motor forward off
    analogWrite(motorRightPin2, rightSpeed);  // PWM for right motor backward
  }
  else if (direction == "R" || direction == "RIGHT"){
    // Turn left (no PID correction needed)
    analogWrite(motorLeftPin1, 0);         // Left motor forward off
    analogWrite(motorLeftPin2, pwmSpeed);  // PWM for left motor backward
    analogWrite(motorRightPin1, pwmSpeed); // PWM for right motor forward
    analogWrite(motorRightPin2, 0);        // Right motor backward off
  }
  else if (direction == "L" || direction == "LEFT") {
    // Turn right (no PID correction needed)
    analogWrite(motorLeftPin1, pwmSpeed);  // PWM for left motor forward
    analogWrite(motorLeftPin2, 0);         // Left motor backward off
    analogWrite(motorRightPin1, 0);        // Right motor forward off
    analogWrite(motorRightPin2, pwmSpeed); // PWM for right motor backward
  }
  else if (direction == "S" || direction == "STOP") {
    // Stop all motors
    analogWrite(motorLeftPin1, 0);
    analogWrite(motorLeftPin2, 0);
    analogWrite(motorRightPin1, 0);
    analogWrite(motorRightPin2, 0);
  }
}

void processCommand(String command) {
  // Remove angle brackets and any newline character
  command.trim();
  
  // Check if command has proper structure
  if (!command.startsWith("<") || !command.endsWith(">") || command.length() < 3) {
    Serial.println("Invalid command format");
    return;
  }
  
  // Remove angle brackets
  command = command.substring(1, command.length() - 1);
  
  // Check if there are at least 2 commas (direction, speed, and at least one joint)
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
  
  // This is just parsing the command - actual motor control happens in loop()
  
  // Skip if there's a problem with the remaining part of the command
  if (command.indexOf("j1,") == -1) {
    // Partial command - just use the car controls
    Serial.print("Partial command - Car: ");
    Serial.print(carDirection);
    Serial.print(",");
    Serial.println(carSpeed);
    return;
  }
  
  // Joint 1 label
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  /* Skip the "j1" label */
  commaIndex = nextCommaIndex + 1;
  
  // Joint 1 value
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  joint1Pos = command.substring(commaIndex, nextCommaIndex).toInt();
  joint1Pos = constrain(joint1Pos, 0, 180);
  joint1Servo.write(joint1Pos);
  commaIndex = nextCommaIndex + 1;
  
  // Joint 2 label
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  /* Skip the "j2" label */
  commaIndex = nextCommaIndex + 1;
  
  // Joint 2 value
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  joint2Pos = command.substring(commaIndex, nextCommaIndex).toInt();
  joint2Pos = constrain(joint2Pos, 0, 180);
  joint2Servo.write(joint2Pos);
  commaIndex = nextCommaIndex + 1;
  
  // Joint 3 label
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  /* Skip the "j3" label */
  commaIndex = nextCommaIndex + 1;
  
  // Joint 3 value
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  joint3Pos = command.substring(commaIndex, nextCommaIndex).toInt();
  joint3Pos = constrain(joint3Pos, 0, 180);
  joint3Servo.write(joint3Pos);
  commaIndex = nextCommaIndex + 1;
  
  // Joint 4 label
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  /* Skip the "j4" label */
  commaIndex = nextCommaIndex + 1;
  
  // Joint 4 value
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  joint4Pos = command.substring(commaIndex, nextCommaIndex).toInt();
  joint4Pos = constrain(joint4Pos, 0, 180);
  joint4Servo.write(joint4Pos);
  commaIndex = nextCommaIndex + 1;
  
  // Joint 5 label
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  /* Skip the "j5" label */
  commaIndex = nextCommaIndex + 1;
  
  // Joint 5 value
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  joint5Pos = command.substring(commaIndex, nextCommaIndex).toInt();
  joint5Pos = constrain(joint5Pos, 0, 180);
  joint5Servo.write(joint5Pos);
  commaIndex = nextCommaIndex + 1;
  
  // Joint 6 label
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  /* Skip the "j6" label */
  commaIndex = nextCommaIndex + 1;
  
  // Joint 6 value
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  joint6Pos = command.substring(commaIndex, nextCommaIndex).toInt();
  joint6Pos = constrain(joint6Pos, 0, 180);
  joint6Servo.write(joint6Pos);
  commaIndex = nextCommaIndex + 1;
  
  // Joint 7 label
  nextCommaIndex = command.indexOf(',', commaIndex);
  if (nextCommaIndex == -1) return;
  /* Skip the "j7" label */
  commaIndex = nextCommaIndex + 1;
  
  // Joint 7 value - last value in the command string
  joint7Pos = command.substring(commaIndex).toInt();
  joint7Pos = constrain(joint7Pos, 0, 180);
  joint7Servo.write(joint7Pos);
  
  // Debug output - confirm command was received
  Serial.print("Command received - Car: ");
  Serial.print(carDirection);
  Serial.print(",");
  Serial.print(carSpeed);
  Serial.print(" | Joints: ");
  Serial.print(joint1Pos);
  Serial.print(",");
  Serial.print(joint2Pos);
  Serial.print(",");
  Serial.print(joint3Pos);
  Serial.print(",");
  Serial.print(joint4Pos);
  Serial.print(",");
  Serial.print(joint5Pos);
  Serial.print(",");
  Serial.print(joint6Pos);
  Serial.print(",");
  Serial.println(joint7Pos);
}

void loop() {
  // Read gyroscope data and calculate angle
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calculate time difference for gyro integration
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;
  
  // Calculate angle from gyro data
  float gyroZ = g.gyro.z - gyroZOffset;  
  angleZ += gyroZ * dt;
  
  // Normalize angle to avoid overflow
  if (angleZ > 6.19) {
    angleZ = 0;
  } else if (angleZ < -6.20) {
    angleZ = 0;
  }
  
  // Convert to degrees for easier understanding
  DegreeAngle = (angleZ / 6.20) * 360;
  
  // Calculate rotational angle in -180 to 180 range
  int RotationalAngle = DegreeAngle;
  if (DegreeAngle > 179) {
    RotationalAngle = DegreeAngle - 360;
  } else if (DegreeAngle < -179) {
    RotationalAngle = DegreeAngle + 360;
  }
  
  // Read from Serial port and process commands
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    receivedCommand += inChar;
    if (inChar == '\n' || inChar == '>') {
      commandComplete = true;
    }
  }
  
  // Process completed command
  if (commandComplete) {
    // Store direction before processing
    String prevDirection = carDirection;
    
    // Process the received command
    processCommand(receivedCommand);
    receivedCommand = "";
    commandComplete = false;
    
    // If direction changed, reset the PID control flags
    if (prevDirection != carDirection) {
      isFDesiredAngleSet = false;
      isBDesiredAngleSet = false;
      isLDesiredAngleSet = false;
      isRDesiredAngleSet = false;
      isSDesiredAngleSet = false;
    }
  }
  
  // Apply PID control based on direction
  float pidOutput = 0;
  
  if (carDirection == "F" || carDirection == "FWD" || carDirection == "FORWARD") {
    if (!isFDesiredAngleSet) { 
      desired_angle = RotationalAngle;  // Set desired angle when first starting to move forward
      resetPID(); // Reset PID values
      isFDesiredAngleSet = true;
      Serial.print("Forward - Locked angle: ");
      Serial.println(desired_angle);
    }
    pidOutput = computePID(desired_angle, RotationalAngle, dt);
    controlCar(carDirection, carSpeed, pidOutput);
  } 
  else if (carDirection == "B" || carDirection == "BWD" || carDirection == "BACKWARD") {
    if (!isBDesiredAngleSet) { 
      desired_angle = RotationalAngle;  // Set desired angle when first starting to move backward
      resetPID(); // Reset PID values
      isBDesiredAngleSet = true;
      Serial.print("Backward - Locked angle: ");
      Serial.println(desired_angle);
    }
    // Note: For backward motion, we invert the PID correction
    pidOutput = computePID(desired_angle, RotationalAngle, dt);
    controlCar(carDirection, carSpeed, pidOutput);
  }
  else {
    // For other directions (LEFT, RIGHT, STOP), use standard control without PID
    controlCar(carDirection, carSpeed);
    
    // Reset flags for the appropriate direction
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
  
  // If direction is not forward, reset forward flag
  if (carDirection != "F" && carDirection != "FWD" && carDirection != "FORWARD") {
    isFDesiredAngleSet = false;
  }
  
  // If direction is not backward, reset backward flag
  if (carDirection != "B" && carDirection != "BWD" && carDirection != "BACKWARD") {
    isBDesiredAngleSet = false;
  }
  
  // Every 500ms, print debug information (gyro angle and PID output)
  static unsigned long lastDebugTime = 0;
  if (currentTime - lastDebugTime > 500) {
    lastDebugTime = currentTime;
    Serial.print("Angle: ");
    Serial.print(RotationalAngle);
    Serial.print(", Desired: ");
    Serial.print(desired_angle);
    if (carDirection == "F" || carDirection == "B") {
      Serial.print(", PID output: ");
      Serial.println(pidOutput);
    } else {
      Serial.println();
    }
  }
  
  // Small delay to prevent CPU overload
  delay(5);
}
