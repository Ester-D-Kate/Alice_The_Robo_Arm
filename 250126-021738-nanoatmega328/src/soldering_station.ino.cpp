# 1 "C:\\Users\\mahaj\\AppData\\Local\\Temp\\tmpvsjirc4o"
#include <Arduino.h>
# 1 "C:/Users/mahaj/Documents/PlatformIO/Projects/250126-021738-nanoatmega328/src/soldering_station.ino"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <max6675.h>



#define ENCODER_CLK_PIN 2
#define ENCODER_DT_PIN 3
#define ENCODER_SW_PIN 4


#define MOSFET_IRON1_PIN 6
#define MOSFET_IRON2_PIN 5
#define MOSFET_IRON3_PIN 7
#define MOSFET_IRON4_PIN A1


#define MAX6675_IRON1_SO_PIN 12
#define MAX6675_IRON1_CS_PIN 10
#define MAX6675_IRON1_SCK_PIN 13
#define MAX6675_IRON2_SO_PIN 9
#define MAX6675_IRON2_CS_PIN 11
#define MAX6675_IRON2_SCK_PIN 13
#define MAX6675_IRON3_SO_PIN 8
#define MAX6675_IRON3_CS_PIN 8
#define MAX6675_IRON3_SCK_PIN 13
#define MAX6675_IRON4_SO_PIN A2
#define MAX6675_IRON4_CS_PIN A2
#define MAX6675_IRON4_SCK_PIN 13


MAX6675 thermocoupleIron1(MAX6675_IRON1_SCK_PIN, MAX6675_IRON1_CS_PIN, MAX6675_IRON1_SO_PIN);
MAX6675 thermocoupleIron2(MAX6675_IRON2_SCK_PIN, MAX6675_IRON2_CS_PIN, MAX6675_IRON2_SO_PIN);
MAX6675 thermocoupleIron3(MAX6675_IRON3_SCK_PIN, MAX6675_IRON3_CS_PIN, MAX6675_IRON3_SO_PIN);
MAX6675 thermocoupleIron4(MAX6675_IRON4_SCK_PIN, MAX6675_IRON4_CS_PIN, MAX6675_IRON4_SO_PIN);


LiquidCrystal_I2C lcd(0x27, 16, 2);


volatile int encoderValue = 0;
int lastEncoded = 0;


int setTemperatureIron1 = 350;
int setTemperatureIron2 = 350;
int setTemperatureIron3 = 350;

float currentTemperatureIron1 = 0.0;
float currentTemperatureIron2 = 0.0;
float currentTemperatureIron3 = 0.0;
float currentTemperatureIron4 = 0.0;

int currentIron = 0;


unsigned long lastTempReadTime = 0;
const unsigned long tempReadInterval = 500;
void setup();
void loop();
void updateEncoder();
void handleEncoderSwitch();
float readTemperature(MAX6675 &thermocouple, bool applyOffset);
void updateLCD();
#line 60 "C:/Users/mahaj/Documents/PlatformIO/Projects/250126-021738-nanoatmega328/src/soldering_station.ino"
void setup() {
  Serial.begin(9600);


  pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  pinMode(MOSFET_IRON1_PIN, OUTPUT);
  pinMode(MOSFET_IRON2_PIN, OUTPUT);
  pinMode(MOSFET_IRON3_PIN, OUTPUT);
  pinMode(MOSFET_IRON4_PIN, OUTPUT);

  digitalWrite(MOSFET_IRON1_PIN, LOW);
  digitalWrite(MOSFET_IRON2_PIN, LOW);
  digitalWrite(MOSFET_IRON3_PIN, LOW);
  digitalWrite(MOSFET_IRON4_PIN, LOW);


  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Soldering Ctrl");


  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT_PIN), updateEncoder, CHANGE);
}

void loop() {
  handleEncoderSwitch();

  if (millis() - lastTempReadTime >= tempReadInterval) {
    lastTempReadTime = millis();


    currentTemperatureIron1 = readTemperature(thermocoupleIron1, true);
    currentTemperatureIron2 = readTemperature(thermocoupleIron2, false);
    currentTemperatureIron3 = readTemperature(thermocoupleIron3, false);
    currentTemperatureIron4 = readTemperature(thermocoupleIron4, false);


    if (currentTemperatureIron1 < setTemperatureIron1) {
      digitalWrite(MOSFET_IRON1_PIN, HIGH);
    } else {
      digitalWrite(MOSFET_IRON1_PIN, LOW);
    }

    if (currentTemperatureIron2 < setTemperatureIron2) {
      digitalWrite(MOSFET_IRON2_PIN, HIGH);
    } else {
      digitalWrite(MOSFET_IRON2_PIN, LOW);
    }

    if (currentTemperatureIron3 < setTemperatureIron3) {
      digitalWrite(MOSFET_IRON3_PIN, HIGH);
    } else {
      digitalWrite(MOSFET_IRON3_PIN, LOW);
    }


    if (currentTemperatureIron4 < setTemperatureIron2) {
      digitalWrite(MOSFET_IRON4_PIN, HIGH);
    } else {
      digitalWrite(MOSFET_IRON4_PIN, LOW);
    }


    updateLCD();
  }
}


void updateEncoder() {
  int MSB = digitalRead(ENCODER_CLK_PIN);
  int LSB = digitalRead(ENCODER_DT_PIN);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    if (currentIron == 0) setTemperatureIron1++;
    if (currentIron == 1) setTemperatureIron2++;
    if (currentIron == 2) setTemperatureIron3++;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    if (currentIron == 0) setTemperatureIron1--;
    if (currentIron == 1) setTemperatureIron2--;
    if (currentIron == 2) setTemperatureIron3--;
  }
  lastEncoded = encoded;
}


void handleEncoderSwitch() {
  static unsigned long lastButtonPress = 0;
  const unsigned long debounceDelay = 50;

  if (digitalRead(ENCODER_SW_PIN) == LOW) {
    if (millis() - lastButtonPress > debounceDelay) {
      lastButtonPress = millis();

      currentIron = (currentIron + 1) % 4;
      while (digitalRead(ENCODER_SW_PIN) == LOW) {

      }
    }
  }
}


float readTemperature(MAX6675 &thermocouple, bool applyOffset) {
  float temperature = thermocouple.readCelsius();
  if (isnan(temperature)) {
    Serial.println("Error: Thermocouple not connected!");
    return 0.0;
  }
  if (applyOffset) {
    temperature += 100;
  }
  return temperature;
}


void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Iron ");
  lcd.print(currentIron + 1);
  lcd.print(" Set: ");
  if (currentIron == 0) lcd.print(setTemperatureIron1);
  if (currentIron == 1) lcd.print(setTemperatureIron2);
  if (currentIron == 2) lcd.print(setTemperatureIron3);
  if (currentIron == 3) lcd.print(setTemperatureIron2);

  lcd.setCursor(0, 1);
  lcd.print("Cur: ");
  if (currentIron == 0) lcd.print(currentTemperatureIron1);
  if (currentIron == 1) lcd.print(currentTemperatureIron2);
  if (currentIron == 2) lcd.print(currentTemperatureIron3);
  if (currentIron == 3) lcd.print(currentTemperatureIron4);
  lcd.print("C");
}