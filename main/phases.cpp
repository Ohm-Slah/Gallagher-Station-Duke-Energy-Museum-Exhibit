/*
 * File name:         "phases.cpp"
 * Contributor(s):    Elliot Eickholtz
 * Last edit:         11/17/21
 * Code usage:
 * This is a file containing all functions used in each of the five phases of the "main.ino" file.
 * 
 */

#include "phases.h"
// Create a new servo object:
Servo myservo;
// Create a variable to store the servo position:
int angle = 0;

void initialization() 
{
  /*
   * This fuction is run once on startup. 
   * This is to simply initialize everything needed.
   */

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
void ServoSetup() //Run these once 
{
 // Attach the Servo variable to a pin:
  myservo.attach(servoPin);
}


void ServoMove (uint8_t Servo)   //Function for the servo motor 
{
  // Tell the servo to go to a particular angle:
  myservo.write(0);
  delay(1000);
  myservo.write(270); // These values can be changed this is random right now 
  delay(1000);
  myservo.write(0);
  delay(1000);
  // Sweep from 0 to 180 degrees:
  for (angle = 0; angle <= 180; angle += 1) {
    myservo.write(angle);
    delay(15);
  }
  // And back from 180 to 0 degrees:
  for (angle = 180; angle >= 0; angle -= 1) 
  {
    myservo.write(angle);
    delay(30);
  }
  delay(1000);
}


