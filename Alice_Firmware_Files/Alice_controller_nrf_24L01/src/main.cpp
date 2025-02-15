#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const int CE = 10;
const int CSN = 9;

RF24 radio(CE, CSN);   // Create RF24 object

const byte address[6] = "00009"; // Address for communication

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);  // Set the address
  radio.setPALevel(RF24_PA_LOW);   // Power level
  radio.stopListening();           // Set module as transmitter
}

void loop() {
  const char text[] = "Hacked!";
  bool success = radio.write(&text, sizeof(text)); // Send data

  if (success) {
    Serial.println("Message sent successfully!");
  } else {
    Serial.println("Message failed to send.");
  }

  delay(1000); // Send message every second
}
