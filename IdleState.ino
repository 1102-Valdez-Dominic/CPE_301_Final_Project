#include <Wire.h>
#include <LiquidCrystal.h>
#include <RTC.h>
#include <DHT.h>

#define GREEN_LED 9

#define FAN_PIN 6
// Register for Pin 9 which is the green LED
volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* ddr_h = (unsigned char*) 0x101;
volatile unsigned char* pin_h = (unsigned char*) 0x100;


// LCD pins
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// RTC module
RTC rtc;

// DHT sensor
#define DHT_PIN 5
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// Water level sensor pin
#define WATER_LEVEL_PIN A0
#define WATER_THRESHOLD 300 // Adjust according to your sensor

// Time variables
unsigned long lastTempHumidityUpdate = 0;
const unsigned long updateInterval = 60000; // Update once per minute

void setup() {
*ddr_h |= (0x01 << 6); // Set PH6 as output

  lcd.begin(16, 2);
  rtc.begin();
  dht.begin();
  
  changeState(IDLE);
}

enum State {DISABLED, IDLE, ERROR, RUNNING};
State currentState = DISABLED;

void loop() {
  switch (currentState) {
    case IDLE:
      idleState();
      break;
  }
}

void idleState() {
  // Monitor water level
  int waterLevel = analogRead(WATER_LEVEL_PIN);
  if (waterLevel < WATER_THRESHOLD) {
    changeState(ERROR);
    return;
  }
  // Green LED is ON in IDLE state
*port_h = (0x01 << 6);
}

