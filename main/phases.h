/*
 * File name:         "phases.h"
 * Contributor(s):    Elliot Eickholtz, Matthew Wrocklage, Jackson Couch
 * Last edit:         11/24/21
 * Code usage:
 * This is an instantiation file for "phases.cpp".
 * Any libraries used or function declarations are located here.
 */

#pragma once
#ifndef SETUP_H
#define SETUP_H

//PINS 11 & 12 CANNOT BE USED FOR PWM

#define servoPin 8 // Define the servo pin:

#define LED_ON_BOARD 13
#define MOTOR_PIN 10

#include <Arduino.h>
#include <Encoder.h>  //https://github.com/PaulStoffregen/Encoder
#include <TM1637.h>   //https://github.com/AKJ7/TM1637

extern volatile byte currentPhase;
extern volatile bool phaseChange;

void initialization();
byte phaseZero();
byte phaseOne();
byte phaseTwo();
byte phaseThree();
byte phaseFour();

bool serialResponse(char com[]);
void ledBlink(byte LED, int Time);
void ledStateChange(byte State);
void failure();
void completion();
void sleep();
void error();

void servoMove(uint16_t position);
void resetPhases();
int8_t encoderRead(char enc);
void initSevenSegment();
void displayDigitalNumber(float value);
void setDCMotor(uint16_t pwmValue);

void test();
#endif
