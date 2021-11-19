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
void ServoMove (unint_8 Servo);
void ServoSetup(); 
void failure();
void completion();
void sleep();
void error();

#endif
