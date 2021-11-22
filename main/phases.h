/*
 * File name:         "phases.h"
 * Contributor(s):    Elliot Eickholtz, Matthew Wrocklage
 * Last edit:         11/22/21
 * Code usage:
 * This is an instantiation file for "phases.cpp".
 * Any libraries used or function declarations are located here.
 */

#ifndef SETUP_H
#define SETUP_H

//include .h file here 
#include <Arduino.h>
#include <Stepper.h>

void initialization();
bool phaseZero();
bool phaseOne();
bool phaseTwo();
bool phaseThree();
bool phaseFour();

bool serialResponse(char com[]);
void ledBlink(byte LED, int Time);
void ledStateChange(byte State);
void failure();
void completion();
void sleep();
void error();



#endif
