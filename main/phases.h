/*
 * File name:         "phases.h"
 * Contributor(s):    Elliot Eickholtz
 * Last edit:         11/17/21
 * Code usage:
 * This is an instantiation file for "phases.cpp".
 * Any libraries used or function declarations are located here.
 */

#ifndef SETUP_H
#define SETUP_H

//.h files here
#include <Arduino.h>
#include <Stepper.h>

void initialization();
bool phaseZero();
bool phaseOne();
bool phaseTwo();
bool phaseThree();
bool phaseFour();


bool serialResponse(char com[]);
void failure();
void completion();
void sleep();
void error();
void ServoSetup(); 
void ServoMove(uint8_t Servo) ; 



#endif
