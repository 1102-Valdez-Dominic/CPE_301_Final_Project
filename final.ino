//Authors: Dominic Valdez, Hamza Syed, 
//Date: 4/18/24

//Includes the Arduino Stepper Library
#include <Stepper.h>

//liquid crystal display library
#include <LiquidCrystal.h>

//DHT library for temp and humidity sensor
#include <dht.h>


// Defines the number of steps per rotation
const int stepsPerRevolution = 2038;
// Creates an instance of stepper class
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);

// LCD pins <--> Arduino pins
const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
//lcd.write()

 #define RDA 0x80
 #define TBE 0x20  
 volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
 volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
 volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
 volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
 volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//Variables for the 1 minute delay for the LCD updates
unsigned long previousMillis = 0;  // will store last time LCD was updated
const long interval = 60000;  // interval at which to update LCD (milliseconds)

void setup() {
  // initialize the serial port on USART0:
  U0init(9600);
  // setup the ADC
  adc_init();
  // Nothing to do for stepper motor (Stepper Library sets pins as outputs)

  // For @buttercup
  //Setup for attach_interupt 
  //pinMode(ledPin, OUTPUT); Change to GPIO of Start button pin
  //pinMode(interruptPin, INPUT_PULLUP); Change to GPIO of interruptPin, USE PIN 2!! 
  //attachInterrupt(digitalPinToInterrupt(interruptPin), ??? , RISING); In the ??? space but the name of the function that is called when button is pressed. The function should basically just be flag variable that changes from 0 to 1 so the state can transition 
}

void loop() {
  //Stepper motor Example code:
    // Rotate CW slowly at 5 RPM
    myStepper.setSpeed(5);
    myStepper.step(stepsPerRevolution);
    //delay(1000); Use custom delay function

    // Rotate CCW quickly at 10 RPM
    myStepper.setSpeed(10);
    myStepper.step(-stepsPerRevolution);
    //delay(1000); Use custom delay function
 

  // The code below uses the millis() function, a command that returns the number of milliseconds since the board started running the sketch
  unsigned long currentMillis = millis(); 

  if (currentMillis - previousMillis >= interval) {
    // save the last time you updated the LCD
    previousMillis = currentMillis;

    // Write code that updates the LCD with Humidity and temperature for all states EXCEPT Disabled. 

    
  }

}



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
  //  Students are responsible for understanding
  //  this initialization code for the ATmega2560 USART0
  //  and will be expected to be able to intialize
  //  the USART in differrent modes.
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

