#include <Arduino.h>  // Include the Arduino core library
#include <Servo.h>  // Include the Servo library

Servo myServo;  // Create a Servo object

const int servoPin = 4; // You can use ANY digital pin, not just PWM pins

void setup() {
  myServo.attach(servoPin);  // Attach servo to the pin
}

void loop() {
  // Move servo from 0 to 180 degrees
  for (int angle = 0; angle <= 180; angle++) {
    myServo.write(0);    // Set the servo position
    delay(15);               // Wait a little bit for smooth movement
  }
  
  delay(500); // Wait half a second at the end

  // Move servo back from 180 to 0 degrees
  for (int angle = 180; angle >= 0; angle--) {
    myServo.write(180);
    delay(15);
  }

  delay(500); // Wait again before repeating
}
