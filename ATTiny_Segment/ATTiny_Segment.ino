#define F_CPU 1000000UL

//#define NONE {1,1,1,1,1,1,1,1}
//#define ZERO {0,0,0,0,0,0,1,1}
//#define ONE {1,0,0,1,1,1,1,1}
//#define TWO {0,0,1,0,0,1,0,1}
//#define THREE {0,0,0,0,1,1,0,1}
//#define FOUR {1,0,0,1,1,0,0,1}
//#define FIVE {0,1,0,0,1,0,0,1}
//#define SIX {0,1,0,0,0,0,0,1}
//#define SEVEN {0,0,0,1,1,1,1,1}
//#define EIGHT {0,0,0,0,0,0,0,1}
//#define NINE {0,0,0,1,1,0,0,1}

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

byte presets[11] = {ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, NONE};

const byte scan[6] = {D1, D2, D3, D4, D5, D6};
const byte segments[8] = {SegA, SegB, SegC, SegD, SegE, SegF, SegG, DP};

//const bool none[] = {1,1,1,1,1,1,1,1};
//const bool zero[] = {0,0,0,0,0,0,1,1};
//const bool one[] = {1,0,0,1,1,1,1,1};
//const bool two[] = {0,0,1,0,0,1,0,1};
//const bool three[] = {0,0,0,0,1,1,0,1};
//const bool four[] = {1,0,0,1,1,0,0,1};
//const bool five[] = {0,1,0,0,1,0,0,1};
//const bool six[] = {0,1,0,0,0,0,0,1};
//const bool seven[] = {0,0,0,1,1,1,1,1};
//const bool eight[] = {0,0,0,0,0,0,0,1};
//const bool nine[] = {0,0,0,1,1,0,0,1};

byte data[6] = {NONE, NONE, NONE, NONE, NONE, NONE};

void setup() {
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
  digitalWrite(scan[0], HIGH);
  
}

void scanAnodes(uint16_t del)
{
  for(int i=0; i<6; i++)
  {
    for (int j=0; j<8; j++)
    {
      digitalWrite(segments[j], (data[i]<<j)&0x80);
    }
    digitalWrite(scan[i], HIGH);
    delay(del);
    digitalWrite(scan[i], LOW);
    delay(1);
    
  }
}

void displayData(uint16_t del)
{
  int count = millis();
  while(count+1000>millis()) scanAnodes(1);
}

void loop() {
  while(1)
  {
    for(int i=0; i<11; i++)
    {
      for(int j=0; j<6; j++) data[j] = presets[(i+j)%11];
      displayData(1000);
    }
  }
}
