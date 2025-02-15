#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const int CE = 9;
const int CSN = 10;

RF24 radio(CE, CSN);   // Create RF24 object

const byte address[6] = "00009"; // Address for communication

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
  }else{
    Serial.println("No message received");
    delay(1000);
  }
}
