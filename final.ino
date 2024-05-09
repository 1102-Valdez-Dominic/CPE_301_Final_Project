
//Authors: Dominic Valdez, Hamza Syed, ZInat Namira
//Date: 5/10/24

//Includes the Arduino Stepper Library
#include <Stepper.h>

//liquid crystal display library
#include <LiquidCrystal.h>

//DHT library for temp and humidity sensor
#include <DHT11.h>
DHT11 dht11(22); //DHT sensor is on pin 22

// Defines the number of steps per rotation
const int stepsPerRevolution = 2038;
// Creates an instance of stepper class
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper myStepper = Stepper(stepsPerRevolution, 46, 50, 48, 52);

// LCD pins <--> Arduino pins
const int RS = 12, EN = 11, D4 = 5, D5 = 4, D6 = 3, D7 = 2;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
//lcd.write()

#define RDA 0x80
#define TBE 0x20  

//Threshold in Celcius
#define Lower_Temp_Threshold 23
#define Upper_Temp_Threshold 25

#define Lower_Water_Threshold 150

//Register Pointers for my_delay function
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

//Pointers for ADC
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//Define Register Pointers for the buttons for vent control. Buttons on pins 26 (PA4) and 27 (PA5).
volatile unsigned char* port_a4 = (unsigned char*) 0x22;
volatile unsigned char* ddr_a4  = (unsigned char*) 0x21;
volatile unsigned char* pin_a4  = (unsigned char*) 0x20;

volatile unsigned char* port_a5 = (unsigned char*) 0x22;
volatile unsigned char* ddr_a5  = (unsigned char*) 0x21;
volatile unsigned char* pin_a5  = (unsigned char*) 0x20;

//Define Register Pointers for the water sensor power, pin 7 (PH4)
volatile unsigned char* port_h4 = (unsigned char*) 0x102;
volatile unsigned char* ddr_h4  = (unsigned char*) 0x101;
volatile unsigned char* pin_h4  = (unsigned char*) 0x100;

//Define Register Pointers for the DC motor, pin 35 (PC2)
volatile unsigned char* port_c2 = (unsigned char*) 0x28;
volatile unsigned char* ddr_c2  = (unsigned char*) 0x27;
volatile unsigned char* pin_c2  = (unsigned char*) 0x26;


//LED GPIO
//GREEN LED pin 9 PH6
volatile unsigned char* port_h6 = (unsigned char*) 0x102;
volatile unsigned char* ddr_h6  = (unsigned char*) 0x101;
volatile unsigned char* pin_h6  = (unsigned char*) 0x100;

//BLUE LED pin 49 PL0
volatile unsigned char* port_l0 = (unsigned char*) 0x10B;
volatile unsigned char* ddr_l0  = (unsigned char*) 0x10A;
volatile unsigned char* pin_l0  = (unsigned char*) 0x109;

//YELLOW LED pin 51 PB2
volatile unsigned char* port_b2 = (unsigned char*) 0x25;
volatile unsigned char* ddr_b2  = (unsigned char*) 0x24;
volatile unsigned char* pin_b2  = (unsigned char*) 0x23;

//RED LED pin 53 PB0
volatile unsigned char* port_b0 = (unsigned char*) 0x25;
volatile unsigned char* ddr_b0  = (unsigned char*) 0x24;
volatile unsigned char* pin_b0  = (unsigned char*) 0x23;

//GPIO for start button pin 18 (PD3)
volatile unsigned char* port_d3 = (unsigned char*) 0x2B;
volatile unsigned char* ddr_d3  = (unsigned char*) 0x2A;
volatile unsigned char* pin_d3  = (unsigned char*) 0x29;

//GPIO for reset button pin 19 (PD2)
volatile unsigned char* port_d2 = (unsigned char*) 0x2B;
volatile unsigned char* ddr_d2  = (unsigned char*) 0x2A;
volatile unsigned char* pin_d2  = (unsigned char*) 0x29;

//GPIO for stop button pin 20 (PD1)
volatile unsigned char* port_d1 = (unsigned char*) 0x2B;
volatile unsigned char* ddr_d1  = (unsigned char*) 0x2A;
volatile unsigned char* pin_d1  = (unsigned char*) 0x29;

//Variables for the 1 minute delay for the LCD updates
unsigned long previousMillis = 0;  // will store last time LCD was updated
const long interval = 60000;  // interval at which to update LCD (milliseconds)

int temperature = 0;
int humidity = 0;
int result = 0; //Variable to store temp and humidity values

int water_value; // variable to store the water sensor value

int state = 0; //Variable to keep track of the states, 0 = disabled, 1 = idle, 2 = error, 3 = running

void setup() {
  // initialize the serial port on USART0:
  U0init(9600);
  // setup the ADC
  adc_init();
 
  //Setup for attach_interupt
  attachInterrupt(digitalPinToInterrupt(18), start , RISING); //Interupt for start button
  attachInterrupt(digitalPinToInterrupt(20), stop , RISING); // Interupt for stop button
 
  //liquid crystal initialization
  lcd.clear();
  lcd.begin(16, 2);// set up number of columns and rows
  lcd.setCursor(0, 0); // move cursor to (0, 0)

  //Setup for Water Sensor
  *ddr_h4 |= (0x01 << 4); //Water sensor power pin as OUTPUT.
  *port_h4 &= ~(0x01 << 4); //Turn the water sensor OFF.

  //Set Pin 26 (PA4) to INPUT for vent button.
  *ddr_a4 &= 0b11101111;
  //Set Pin 27 (PA5) to INPUT for vent button.
  *ddr_a5 &= 0b11011111;

  //Set Motor pin as OUTPUT
  *ddr_c2 |=(0x01 << 2);

  //setup LED GPIO
  //set pl0 to OUTPUT (BLUE)
  *ddr_l0 |= (0x01 << 0);
  //set Ph6 to OUTPUT (GREEN)
  *ddr_h6 |= (0x01 << 6);
  //set Pb0 to OUTPUT (RED)
  *ddr_b0 |= (0x01 << 0);
  //set pb2 to OUTPUT (YELLOW)
  *ddr_b2 |= (0x01 << 2);

  //Setup ISR pin as INPUT
  *ddr_d3 &= ~(0x01 << 3);

  //Setup reset button pin as INPUT
  *ddr_d2 &= ~(0x01 << 2);

  //Setup stop button pin as INPUT
  *ddr_d1 &= ~(0x01 << 1);

  //begin in disabled state
  state = 0;
  
}

void loop() {

  //DISABLED
  if(state == 0){
    //yellow LED ON
    *port_b2 |= (0x01 << 2);
    //other LEDS OFF
    *port_b0 &= ~(0x01 << 0);
    *port_h6 &= ~(0x01 << 6);
    *port_l0 &= ~(0x01 << 0);
    
    //Turn off motor
    *port_c2 &= ~(0x01 << 2);

    my_delay(1000);      
  }
  //IDLE
  if(state == 1){
    //green LED ON
    *port_h6 |= (0x01 << 6);
    //other LEDS OFF
    *port_b2 &= ~(0x01 << 2);
    *port_b0 &= ~(0x01 << 0);
    *port_l0 &= ~(0x01 << 0);

    //Turn off motor
    *port_c2 &= ~(0x01 << 2);

    //Monitor Water Level
    *port_h4 |= (0x01 << 4); //Turn the water sensor ON.
    unsigned int water_level = adc_read(5); // read the analog value from sensor
    printvolt(water_level);
    U0putchar('V');
    U0putchar('\n');
    my_delay(1000);

    Humidity_Temp_Monitor(); //Monitor temperature and humidity on LCD every minute
    VentControl(); //Allow vent position control with two buttons
    
    if(temperature > Upper_Temp_Threshold){
      state = 3;
    }
    if(water_value <= Lower_Water_Threshold){
      state = 2;
    }

    my_delay(1000);      
  }
  //ERROR
  if(state == 2){
    //using LCD to display error message:
    lcd.setCursor(0,0); 
    lcd.print("Water level is too low");

    //red LED ON
    *port_b0 |= (0x01 << 0);
    //other LEDS OFF
    *port_b2 &= ~(0x01 << 2);
    *port_h6 &= ~(0x01 << 6);
    *port_l0 &= ~(0x01 << 0);

    //Turn off motor
    *port_c2 &= ~(0x01 << 2);

    Humidity_Temp_Monitor(); //Monitor temperature and humidity on LCD every minute
    VentControl(); //Allow vent position control with two buttons
 
    if(*pin_d2 & (0x01 << 2)){ //Reset button pressed
      state = 1;
    }

    my_delay(1000);      
  }
  //RUNNING
  if(state == 3){
    //Blue LED ON
    *port_l0 |= (0x01 << 0);
    //other LEDS OFF
    *port_b2 &= ~(0x01 << 2);
    *port_b0 &= ~(0x01 << 0);
    *port_h6 &= ~(0x01 << 6);

    //Turn on motor
    *port_c2 |=(0x01 << 2); //motor pin high

    //Monitor Water Level
    *port_h4 |= (0x01 << 4); //Turn the water sensor ON.
    unsigned int water_level = adc_read(5); // read the analog value from sensor
    printvolt(water_level);
    U0putchar('V');
    U0putchar('\n');
    my_delay(10);

    Humidity_Temp_Monitor(); //Monitor temperature and humidity on LCD every minute
    VentControl(); //Allow vent position control with two buttons

    if(water_level < Lower_Water_Threshold){
     state = 2;
    }
    if(temperature <= Lower_Temp_Threshold){
      state = 1;
    }
    my_delay(1000);   
  }

  
}

//---------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------------------------------------//
//Functions

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void U0init(unsigned long U0baud)
{
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

unsigned char U0kbhit()
{
  // Read USART0 RDA status bit and return non-zero true if set
  return (*myUCSR0A & RDA);
}

unsigned char U0getchar()
{
  // Read input character from USART0 input buffer
  unsigned char ch;
  ch = *myUDR0;
  return ch;
 
}

void U0putchar(unsigned char U0pdata)
{
  // Wait for USART0 (myUCSR0A) TBE to be set then write character to
  // transmit buffer
  while ((*myUCSR0A & TBE)==0){};
  *myUDR0 = U0pdata;
}

void my_delay(unsigned int freq)
{
  // calc period
  double period = 1.0/double(freq);
  // 50% duty cycle
  double half_period = period/ 2.0f;
  // clock period def
  double clk_period = 0.0000000625;
  // calc ticks
  unsigned int ticks = half_period / clk_period;
  // stop the timer
  *myTCCR1B &= 0xF8;
  // set the counts
  *myTCNT1 = (unsigned int) (65536 - freq);
  // start the timer
  * myTCCR1A = 0x0;
  * myTCCR1B |= 0b00000001;
  // wait for overflow
  while((*myTIFR1 & 0x01)==0); // 0b 0000 0000
  // stop the timer
  *myTCCR1B &= 0xF8;   // 0b 0000 0000
  // reset TOV          
  *myTIFR1 |= 0x01;
  //delay(freq - 500); //test with old (remove after)
}

void printvolt(unsigned int analogValue){
  unsigned char flag = 0;
  if(analogValue >= 1000){
    U0putchar(analogValue/1000 + '0');
    flag = 1;
    analogValue = analogValue % 1000;
  }
  U0putchar('.');
  if(analogValue >= 100 || flag == 1){
    U0putchar(analogValue/100 + '0');
    flag = 1;
    analogValue = analogValue % 100;
  }
  if(analogValue >= 10 || flag == 1){
    U0putchar(analogValue/10 + '0');
    flag = 1;
    analogValue = analogValue % 10;
  }
  U0putchar(analogValue + '0');

}

void start(){
  if(state == 0){
  state = 1;
  }
}

void stop(){
 if(state != 0){
    state = 0;
  }
}

void Humidity_Temp_Monitor(){
  // The code below uses the millis() function, a command that returns the number of milliseconds since the board started running the sketch
  unsigned long currentMillis = millis();

  //Updates the LCD every minute with humidity and temperature for all states EXCEPT Disabled.
  if (currentMillis - previousMillis >= interval) {
    // save the last time you updated the LCD
    previousMillis = currentMillis;
 
    // Attempt to read the temperature and humidity values from the DHT11 sensor.
    result = dht11.readTemperatureHumidity(temperature, humidity);
    my_delay(1000);
    //using LCD to display humidity and temp:
    lcd.setCursor(0,0);
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print((char)223);
    lcd.print("C");
    lcd.setCursor(0,1);
    lcd.print("Humidity: ");
    lcd.print(humidity);
    lcd.print("%");
    
  }
}

void VentControl(){
  //Checks to see if one of the two vent buttons are pushed then rotates the stepper motor depending on which button is pushed
  if((*pin_a4 & 0b00010000) ){
    // Rotate CW at 10 RPM if pin 26 button is high
    myStepper.setSpeed(10);
    myStepper.step(stepsPerRevolution);
    my_delay(1000); 
  }
  else if((*pin_a5 & 0b00100000) ){
    // Rotate CCW at 10 RPM if pin 27 button is high
    myStepper.setSpeed(10);
    myStepper.step(-stepsPerRevolution);
    my_delay(1000); 
  }
}