#include <Wire.h>
#include <LiquidCrystal.h>
#include <RTC.h>
#include <DHT.h>

#define GREEN_LED 9

#define FAN_PIN 6

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
  pinMode(GREEN_LED, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);

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
    // Other states can be handled here...
  }
}

void idleState() {
  // Monitor water level
  int waterLevel = analogRead(WATER_LEVEL_PIN);
  if (waterLevel < WATER_THRESHOLD) {
    changeState(ERROR);
    return;
  }
  
  // Monitor temperature and humidity
  unsigned long currentMillis = millis();
  if (currentMillis - lastTempHumidityUpdate >= updateInterval) {
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    // Display on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.print(humidity);
    
    lastTempHumidityUpdate = currentMillis;
  }
  
  // Green LED is ON in IDLE state
  digitalWrite(GREEN_LED, HIGH);
}

void changeState(State newState) {
  // Turn off all LEDs
  digitalWrite(GREEN_LED, LOW);
  
  currentState = newState;
  switch (currentState) {
    case IDLE:
      digitalWrite(GREEN_LED, HIGH);
      // Log the state transition with the real-time clock
      rtc.now().printToSerial();
      Serial.println(" - Transitioned to IDLE state");
      break;
    // Handle other states...
  }
}
