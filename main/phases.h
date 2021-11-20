/*
 * File name:         "phases.h"
 * Contributor(s):    Elliot Eickholtz, Andrew Boehm, jackson Couch
 * Last edit:         11/18/21
 * Code usage:
 * This is an instantiation file for "phases.cpp".
 * Any libraries used or function declarations are located here.
 */

#ifndef SETUP_H
#define SETUP_H

#include <Arduino.h>
#include <Encoder.h>
#include <TM1637.h>


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
void encoderRead(uint8_t Encoder);
void initSevenSegment();
void displayDigitalNumber(float value);

#endif
