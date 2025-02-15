/* If your serial output has these values same
 then Your nrf24l01 module is in working condition :
  EN_AA = 0x3f
  EN_RXADDR = 0x02 
  RF_CH = 0x4c 
  RF_SETUP = 0x03 
  CONFIG = 0x0f 
*/
#include <Arduino.h>
#include <SPI.h> 
#include <RF24.h> 
#include <printf.h> 

RF24 radio(9, 10); 

byte addresses[][6] = {"1Node", "2Node"}; 

void setup() { 
  radio.begin(); 
  radio.setPALevel(RF24_PA_LOW); 
  radio.openWritingPipe(addresses[0]); 
  radio.openReadingPipe(1, addresses[1]); 
  radio.startListening(); 
  Serial.begin(9600); 
  printf_begin(); 
  radio.printDetails(); 
  } 

  void loop() {
    
    }