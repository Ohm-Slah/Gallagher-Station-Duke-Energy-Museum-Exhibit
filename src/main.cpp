/*
 * File name:         "main.cpp"
 * Contributor(s):    Elliot Eickholtz
 * Last edit:         4/15/22
 * Code usage:
 * This is the main cpp file used in the creation of a museum exhibit of the Gallagher Station is New Albany, Indiana. 
 * The intended usage of this code is to simulate the responsibilities of a control station operator of a coal-fired power plant, 
 * while keeping the content and interactions approachable for a person with little-to-no experience in the field. 
 * This code is sequential, and can be subdivided into 5 main sections represented numerically from 'zero' to 'four',
 * followed by a 'completion' portion.
 * 
 * Full code Repository:
 * https://github.com/Ohm-Slah/Gallagher-Station-Duke-Energy-Museum-Exhibit
 */

#include "phases.h"

// this function is called once on startup and nerver runs again
void setup() 
{
  initialization();
}

// TODO too many repeats of the same thing, simplify this
// this function is called directly after "void setup()", and it loops forever.
void loop()
{  
  switch(currentPhase)              // switch to case below depending on currentPhase value
  {
    case 0:                         // phase 0
      lastResponse = millis();      // reset sleep timer value
      phaseChange = false;          // reset main switch interrupt trigger variable
      phaseChangeLEDState(0);       // set LED states to Phase 0
      currentPhase = phaseZero();   // set the next phase to what phaseZero() returns. 0=Phase 0, 1=Phase 1, etc...
    break;                          // end of phase 0
    case 1:                         // repeat near identical steps for phase 1
      lastResponse = millis();
      reset();
      intro();                      // play intro video
      phaseChange = false;
      phaseChangeLEDState(1);
      currentPhase = phaseOne();
    break;
    case 2:                         // repeat near identical steps for phase 2
      lastResponse = millis();
      phaseChange = false;
      phaseChangeLEDState(2);
      currentPhase = phaseTwo();
    break;
    case 3:                         // repeat near identical steps for phase 3
      lastResponse = millis();
      phaseChange = false;
      phaseChangeLEDState(3);
      currentPhase = phaseThree();
    break;
    case 4:                         // repeat near identical steps for phase 4
      lastResponse = millis();
      phaseChange = false;
      phaseChangeLEDState(4);
      currentPhase = phaseFour();
    break;
    case 10:                        // repeat near identical steps for completion
      lastResponse = millis();
      phaseChange = false;
      phaseChangeLEDState(10);
      currentPhase = completion();
    break;
    default:                        // when currentPhase is none of the values above
      error();
    break;
  }
}
