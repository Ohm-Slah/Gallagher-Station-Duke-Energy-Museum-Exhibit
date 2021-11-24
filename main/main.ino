/*
 * File name:         "main.ino"
 * Contributor(s):    Elliot Eickholtz
 * Last edit:         11/17/21
 * Code usage:
 * This is the main Arduino file used in the creation of a museum exhibit of the Gallagher Station is New Albany, Indiana. 
 * The intended usage of this code is to simulate the responsibilities of a control station operator of a coal-fired power plant, 
 * while keeping the content and interactions approachable for a person with little-to-no experience in the field. 
 * This code is sequential, and can be subdivided into 5 main sections represented numerically from 'zero' to 'four'.
 */

#include "phases.h"



void setup() 
{
  
  initialization();
  initSevenSegment();
}

void loop() 
{
  zeroPassed = phaseZero();
  
  if (phaseZero())
  {
    onePassed = phaseOne();
    if (phaseOne())
    {
      twoPassed = phaseOne();
      if (phaseTwo())
      {
        threePassed = phaseThree();
        if (phaseThree())
        {
          fourPassed = phaseFour();
          if (phaseFour())
          {
            completion();
          } else {
            failure();
          }
        } else {
          failure();
        }
      } else {
        failure();
      }
    } else {
      failure();
    }
  } else {
    failure();
  }
  
}
