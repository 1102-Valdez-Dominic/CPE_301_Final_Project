#include <Wire.h>
#include <LiquidCrystal.h>
#include <RTC.h>
#include <DHT.h>

// Define pointer-based registers
// Green LED pin
volatile unsigned char* GREEN_LED = (volatile unsigned char*) 0x102;

// Stepper motor pins
volatile unsigned char* stepperPins[] = {(volatile unsigned char*) 0x2E, (volatile unsigned char*) 0x32, (volatile unsigned char*) 0x30, (volatile unsigned char*) 0x34};

// Button pins
volatile unsigned char* buttonPins[] = {(volatile unsigned char*) 0x1A, (volatile unsigned char*) 0x1B};

// LCD pins
const int rs = 2, en = 3, d4 = 4, d5 = 5, d6 = 11, d7 = 12;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// RTC module
RTC rtc;

// DHT sensor
volatile unsigned char* DHT_PIN = (volatile unsigned char*) 22;
volatile unsigned char* DHT_TYPE = (volatile unsigned char*) 11;
DHT dht((uint8_t) DHT_PIN, (uint8_t) DHT_TYPE);

// Water level sensor pins
volatile unsigned char* WATER_LEVEL_PIN = (volatile unsigned char*) 0x10; // Analog Pin 5
const int WATER_THRESHOLD = 300; // Adjust according to sensor

// Time variables
unsigned long lastTempHumidityUpdate = 0;
const unsigned long updateInterval = 60000; // Update once per minute

// Register manipulation pointers for Pin 9 (Green LED)
volatile unsigned char* port_h = (volatile unsigned char*) 0x102;
volatile unsigned char* ddr_h = (volatile unsigned char*) 0x101;
volatile unsigned char* pin_h = (volatile unsigned char*) 0x100;

// Idle state start time
unsigned long idleStartTime = 0;

// LCD update interval
const unsigned long lcdUpdateInterval = 1000; // Update LCD every second
unsigned long lastLcdUpdateTime = 0;

void setup() {
    *ddr_h |= (0x01 << 6); // Set PH6 as output

    lcd.begin(16, 2);
    rtc.begin();
    dht.begin();

    changeState(IDLE);
}

enum State { DISABLED, IDLE, ERROR, RUNNING };
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
    int waterLevel = analogRead((uint8_t)WATER_LEVEL_PIN);
    if (waterLevel < WATER_THRESHOLD) {
        changeState(ERROR);
        return;
    }

    // Green LED is ON in IDLE state
    *port_h |= (0x01 << 6);
    
    // Update LCD display with real-time timestamp and idle state duration
    updateLcd();
}

void updateLcd() {
    unsigned long currentMillis = millis();
    
    // Update LCD every second
    if (currentMillis - lastLcdUpdateTime >= lcdUpdateInterval) {
        lastLcdUpdateTime = currentMillis;
        
        // Get current date and time from RTC
        DateTime now = rtc.now();
        
        // Format current time as a string (HH:MM:SS)
        char currentTime[9];
        snprintf(currentTime, sizeof(currentTime), "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
        
        // Calculate idle time duration
        unsigned long idleDuration = (currentMillis - idleStartTime) / 1000;
        unsigned long idleMinutes = idleDuration / 60;
        unsigned long idleSeconds = idleDuration % 60;
        
        // Display the current time on the first line of the LCD
        lcd.setCursor(0, 0);
        lcd.print(currentTime);
        
        // Display the idle time duration on the second line of the LCD
        lcd.setCursor(0, 1);
        lcd.print("Idle: ");
        lcd.print(idleMinutes);
        lcd.print("m ");
        lcd.print(idleSeconds);
        lcd.print("s");
    }
}

