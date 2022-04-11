/*
   ya yeet
   This code is for the slave device to receive 'n' bits of data from an i2c bus defined by
   the master device.
   Data requests happen every second and print hello to serial monitor
   see file masterI2C for rest of code
*/

#define F_CPU 8000000 //this is for the attiny 4313

#include <Wire.h>

#include <Arduino.h>

// define bytes for 7-segment display to replace these words
#define NONE  0b11111111
#define ZERO  0b00000011
#define ONE   0b10011111
#define TWO   0b00100101
#define THREE 0b00001101
#define FOUR  0b10011001
#define FIVE  0b01001001
#define SIX   0b01000001
#define SEVEN 0b00011111
#define EIGHT 0b00000001
#define NINE  0b00011001
#define A     0b00010001
#define C     0b01100011
#define H     0b10010001
#define Z     0b00100101

// define I/O pins tied to 7-segment anodes
#define D1 7
#define D2 6
#define D3 5
#define D4 2
#define D5 4
#define D6 3

// define I/O pins tied to 7-segment cathodes
#define SegA  9
#define SegB 10
#define SegC 11
#define SegD 12
#define SegE 13
#define SegF 1  //14
#define SegG 15
#define DP   0  //16

// define array of presets to scroll through
byte presets[15] = {ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, NONE, A, C, H, Z};

// define order of annodes tied to their I/O that needs to be multiplexed
const byte scan[6] = {D1, D2, D3, D4, D5, D6};
// define order of segments tied to their I/O
const byte segments[8] = {SegA, SegB, SegC, SegD, SegE, SegF, SegG, DP};

// array to hold data that will be scrolled through to multiplex
char data[6] = {NONE, NONE, NONE, NONE, NONE, NONE};
//char data[6] = {ZERO, ZERO, ZERO, ZERO, ZERO, ZERO};
//char data[6] = {ONE, ONE, ONE, ONE, ONE, ONE};
//char data[6] = {ZERO, TWO, FOUR, ONE, ONE, ONE};

void setup()
{
   Wire.begin(8);                // join i2c bus with address #8
   Wire.onReceive(receiveEvent); // register event
   Serial.begin(9600);           // starts serial output

    for(int i=3; i<8; i++)
    {
      pinMode(i, OUTPUT);
      digitalWrite(i, LOW);
    }
    
    for(int i=9; i<16; i++)
    {
      pinMode(i, OUTPUT);
      digitalWrite(i, HIGH);
    }
    pinMode(0, OUTPUT);
    digitalWrite(0, HIGH);
    pinMode(1, OUTPUT);
    digitalWrite(1, HIGH);
  
}

void loop()
{
   scanAnodes(300);
}

void scanAnodes(uint16_t del)
{
  // push currently saved data onto the 7-segment LEDs, multiplexing them once
 
  for(uint8_t i=0; i<6; i++)
  {
    for (uint8_t j=0; j<8; j++)
    {
      
      digitalWrite(segments[j], (presets[data[i]]) & (0x80 << j));
     
    }
    
    //on and off anodes segments 
    digitalWrite(scan[i], HIGH);
    delayMicroseconds(del);
    digitalWrite(scan[i], LOW);
    
  }
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()

void receiveEvent(int howMany) //contains the amount of bytes
{
  int count = 0;

  while(1 < Wire.available())
  {
    byte c = Wire.read(); //grabs a byte at a time 
    data[count] = c; 
    count++; 
    if (count > 5)
    {
      break; 
    }
  }
  for(; count<6; count++)
  {
   
    data[count] = TWO;
  }
}
