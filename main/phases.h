

/*
 * File name:         "phases.h"
 * Contributor(s):    Elliot Eickholtz
 * Last edit:         11/19/21
 * Code usage:
 * This is an instantiation file for "phases.cpp".
 * Any libraries used or function declarations are located here.
 */

#ifndef SETUP_H
#define SETUP_H

#define LED_ON_BOARD 13
#define MOTOR_PIN 12

#include <Arduino.h>
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
void initSevenSegment();
void displayDigitalNumber(float value);
void setDCMotor(uint16_t pwmValue);

#endif
