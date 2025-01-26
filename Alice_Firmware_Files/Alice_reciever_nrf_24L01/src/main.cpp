#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);  // Create RF24 object

const byte address[6] = "10001"; // Address for communication

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address); // Set the address
  radio.setPALevel(RF24_PA_LOW);     // Power level
  radio.startListening();            // Set module as receiver
}

void loop() {
  if (radio.available()) {
    char text[32] = ""; // Buffer to store received data
    radio.read(&text, sizeof(text)); // Read data
    Serial.print("Received message: ");
    Serial.println(text); // Print received message
    delay(1000);
  }
}
