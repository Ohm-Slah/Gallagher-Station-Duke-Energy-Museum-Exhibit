#include <Arduino.h>

#define F_CPU 1000000UL

#define NONE    0b11111111
#define ZERO    0b00000011
#define ONE     0b10011111
#define TWO     0b00100101
#define THREE   0b00001101
#define FOUR    0b10011001
#define FIVE    0b01001001
#define SIX     0b01000001
#define SEVEN   0b00011111
#define EIGHT   0b00000001
#define NINE    0b00011001
#define ZEROP   0b00000010
#define ONEP    0b10011110
#define TWOP    0b00100100
#define THREEP  0b00001100
#define FOURP   0b10011000
#define FIVEP   0b01001000
#define SIXP    0b01000000
#define SEVENP  0b00011110
#define EIGHTP  0b00000000
#define NINEP   0b00011000
#define A       0b00010001
#define C       0b01100011
#define H       0b10010001
#define Z       0b00100101

#define D1 7
#define D2 6
#define D3 5
#define D4 2
#define D5 4
#define D6 3

#define SegA  9
#define SegB 10
#define SegC 11
#define SegD 12
#define SegE 13
#define SegF 14
#define SegG 15
#define DP   16

byte presets[25] = {ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, NONE,
                    ZEROP,ONEP,TWOP,THREEP,FOURP,FIVEP,SIXP,SEVENP,EIGHTP,NINEP,
                    A, C, H, Z};

const byte scan[6] = {D1, D2, D3, D4, D5, D6};
const byte segments[8] = {SegA, SegB, SegC, SegD, SegE, SegF, SegG, DP};

char data[6] = {NONE, NONE, NONE, NONE, NONE, NONE};

void setup() {
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
  //delay(1000);
  //digitalWrite(scan[0], HIGH);
  
}

void loop() { 
  scanAnodes(400);
  //data[0]++;
  getSerial();
  //testSegments();
}

void testSegments()
{
  for(int i=0; i<6; i++)
  {
    for(int j=0; j<8; j++)
    {
      for(int k=0; k<8; k++)
      {
        
        if(k==j)  digitalWrite(segments[k], LOW);
        else      digitalWrite(segments[k], HIGH);
      }
      delay(100);
    }
    for(int h=0; h<6; h++)
    {
      if(i==h)  digitalWrite(scan[h], HIGH);
      else      digitalWrite(scan[h], LOW);
    }
  }
}

void scanAnodes(uint16_t del)
{
  for(int i=0; i<6; i++)
  {
    for (int j=0; j<8; j++)
    {
      digitalWrite(segments[j], (presets[data[i]])&(0x80>>j));
    }
    digitalWrite(scan[i], HIGH);
    delayMicroseconds(del);
    digitalWrite(scan[i], LOW);
    
  }
}

void getSerial()
{
  char buff[6];
  
  if(Serial.available())
  {
    //for(int i=0; i<6; i++) data[i] = NONE;
    uint8_t len = Serial.readBytesUntil(0x0D, data, 6);
    for(int j=0; j<len; j++)  data[j] = data[j] - '0';
    for(int i=len; i<6; i++) data[i] = NONE;
    Serial.println(len);

    Serial.readBytes(buff, 10);
    return true;
  } else {
    return false;
  }
}
