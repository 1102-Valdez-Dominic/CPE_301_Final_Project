//Authors: Dominic Valdez, Hamza Syed, 
//Date: 4/18/24

 #define RDA 0x80
 #define TBE 0x20  
 volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
 volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
 volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
 volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
 volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

void setup() {
  // initialize the serial port on USART0:
  U0init(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

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