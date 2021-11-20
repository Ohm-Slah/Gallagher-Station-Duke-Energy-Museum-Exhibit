/*
 * File name:         "phases.cpp"
 * Contributor(s):    Elliot Eickholtz, Jackson Couch, Andrew Boehm
 * Last edit:         11/18/21
 * Code usage:
 * This is a file containing all functions used in each of the five phases of the "main.ino" file.
 * 
 */

#include "phases.h"

long oldPosition  = -999; //Sets the Encoder from its original position
Encoder myEnc(2, 3); //Creates an Encoder object, using 2 pins. Creates mulitple Encoder objects, where each uses its own 2 pins. The first pin should be capable of interrupts. If both pins have interrupt capability, both will be used for best performance. 
                    //Encoder will also work in low performance polling mode if neither pin has interrupts. 

TM1637 tm(2, 3); //library instantiation for 7-segment display

void initialization() 
{
  /*
   * This fuction is run once on startup. 
   * This is to simply initialize everything needed.
   */
  pinMode(13, OUTPUT); //LED pin of Arduino Mega
  Serial.begin(9600);
  delay(100);
  
  if (!serialResponse("RESPOND")) error();
}

bool phaseZero() 
{
  /*
   * This fuction is the 'awaiting user input' phase. This will run for a maximum of three hours,
   * and will then enter a sleep state. The only difference in the sleep state is what the raspberry pi displays.
   * When exiting the sleep state, homing everthing like the initialization stage is needed.
   */
  if (!serialResponse("PHASE ZERO")) error();
  delay(1000);
  return true;

}

bool phaseOne() 
{
  /*
   * This function is the first phase of the display.
   */
  if (!serialResponse("PHASE ONE")) error();
  delay(1000);
  return true;
}

bool phaseTwo() 
{
  /*
   * This function is the second phase of the display.
   */
  if (!serialResponse("PHASE TWO")) error();
  delay(1000);
  return true;
}

bool phaseThree() 
{
  /*
   * This function is the third phase of the display.
   */
  if (!serialResponse("PHASE THREE")) error();
  delay(1000);
  return true;
}

bool phaseFour() 
{
  /*
   * This function is the fourth phase of the display.
   */
  if (!serialResponse("PHASE FOUR")) error();
  delay(1000);
  return true;
}

bool serialResponse(char com[])
{
  /*
   * This function takes a predefined string command and confirms a serial response from the raspberry pi running processing.
   * Predefined commands: "RESPOND" "PHASE ZERO" "PHASE ONE" "PHASE TWO" "PHASE THREE" "PHASE FOUR" "FAILURE" "COMPLETE"
   */
  uint8_t attempts = 0;
 
  delay(100);
  while(attempts <= 5)
  {
    Serial.println(com);
    if (Serial.available())
    {
      char val = Serial.read();
      Serial.println(val);
      if (val == '1')
      {
        return true;
      } else {
        return false;
        attempts++;
      }
    }
    delay(100);
    attempts++;
  }
  return false;
  
}

void failure() 
{
  /*
   * This function is the failure state of the display.
   */
  if (!serialResponse("FAILURE")) error();
  delay(1000);
  return true;
}

void completion() 
{
  /*
   * This function is the completion state of the display.
   */
  if (!serialResponse("COMPLETE")) error();
  delay(1000);
  return true;
}

void error()
{
  /*
   * This function is the error state of the display. Only call this if a reset is necessary.
   */
  Serial.println("CRITICAL ERROR");
  while(1)
  {
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
  }
}

void encoderRead(uint8_t Encoder) 
{
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) 
  {
    oldPosition = newPosition;
   // Serial.println(newPosition);
  }
  
}

void initSevenSegment()
{
  /*
   * This function runs once at startup and initializes the 7-segment display.
   */
  tm.begin();
  tm.setBrightness(4);
}

void displayDigitalNumber(float value)
{
  /*
   * This function takes in a 4-digit value, integer or float, and displays it on the 7-segment display.
   * Due to it's simplicity, it may be removed at a later date.
   */
  
  tm.display(value);

}
