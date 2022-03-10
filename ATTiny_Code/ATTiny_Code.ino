/*
 * File name:         "ATTiny_Code.ino"
 * Contributor(s):    Elliot Eickholtz
 * Last edit:         3/10/22
 * Code usage:
 * This code is compiled and linked on the Arduino IDE.
 * It is meant to be run on an ATTiny4313 microcontroller, and is intended
 * to control a unique part of an old 7-segment display, which is full of 
 * solely passive components. More information can be found in the project
 * TDP for specific applications.
 * 
 * Full code Repository:
 * https://github.com/Ohm-Slah/Gallagher-Station-Duke-Energy-Museum-Exhibit
 */

// Define CPU clockspeed
#define F_CPU 8000000UL

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
#define SegF 14
#define SegG 15
#define DP   16

// define array of presets to scroll through
byte presets[11] = {ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, NONE};

// define order of annodes tied to their I/O that needs to be multiplexed
const byte scan[6] = {D1, D2, D3, D4, D5, D6};
// define order of segments tied to their I/O
const byte segments[8] = {SegA, SegB, SegC, SegD, SegE, SegF, SegG, DP};

// array to hold data that will be scrolled through to multiplex
char data[6] = {NONE, NONE, NONE, NONE, NONE, NONE};

void setup()  //only runs once at startup.
{
  // basic initialization
  Serial.begin(9600);
  Serial.setTimeout(5);
  
  for(int i=2; i<8; i++)
  {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
  
  for(int i=9; i<17; i++)
  {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
  }
  
}

void loop() //constantly runs repeatedly
{
  // sweep trough the annodes with data array 1ms between each
  scanAnodes(1);
  // check serial port for incoming data, replace current data if any
  getSerial();
}

void scanAnodes(uint16_t del)
{
  // push currently saved data onto the 7-segment LEDs, multiplexing them once

  for(int i=0; i<6; i++)
  {
    for (int j=0; j<8; j++)
    {
      digitalWrite(segments[j], (presets[int(data[i]-'0')]<<j)&0x80);
    }
    digitalWrite(scan[i], HIGH);
    delay(del);
    digitalWrite(scan[i], LOW);
    delay(1);
    
  }
}

void getSerial()
{
  // checks for serial data at buffer, replaces current data array with new data

  char buff[6];
  
  if(Serial.available())
  {
    //for(int i=0; i<6; i++) data[i] = NONE;
    uint8_t len = Serial.readBytesUntil(0x0A, data, 6);

    for(int i=len; i<6; i++) data[i] = NONE;

    Serial.readBytes(buff, 100);
  }
}
