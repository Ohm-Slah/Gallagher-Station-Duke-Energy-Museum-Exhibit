/*
 * File name:         "main.cpp"
 * Contributor(s):    Elliot Eickholtz
 * Last edit:         2/8/22
 * Code usage:
 * This is the main cpp file used in the creation of a museum exhibit of the Gallagher Station is New Albany, Indiana. 
 * The intended usage of this code is to simulate the responsibilities of a control station operator of a coal-fired power plant, 
 * while keeping the content and interactions approachable for a person with little-to-no experience in the field. 
 * This code is sequential, and can be subdivided into 5 main sections represented numerically from 'zero' to 'four'.
 * 
 * Full code Repository:
 * https://github.com/Ohm-Slah/Gallagher-Station-Duke-Energy-Museum-Exhibit
 */

#include "phases.h"
//#include <Arduino.h>
void setup() 
{
  //Serial.begin(9600);
  //while(1) Serial.println("TEST");
  initialization();
  
}

void loop()
{  
  switch(currentPhase)
  {
    case 0:
      reset();
      lastResponse = millis();
      phaseChange = false;
      phaseChangeLEDState(0);
      currentPhase = phaseZero();
    break;
    case 1:
      phaseChange = false;
      phaseChangeLEDState(1);
      currentPhase = phaseOne();
    break;
    case 2:
      phaseChange = false;
      phaseChangeLEDState(2);
      currentPhase = phaseTwo();
    break;
    case 3:
      phaseChange = false;
      phaseChangeLEDState(3);
      currentPhase = phaseThree();
    break;
    case 4:
      phaseChange = false;
      phaseChangeLEDState(4);
      currentPhase = phaseFour();
    break;
    case 10:
      phaseChange = false;
      phaseChangeLEDState(10);
      currentPhase = completion();
    break;
    default:
      error();
    break;
    
  }
  
}
