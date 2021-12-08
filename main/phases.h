/*
 * File name:         "phases.h"
 * Contributor(s):    Elliot Eickholtz, Matthew Wrocklage, Jackson Couch, Andrew Boehm
 * Last edit:         12/6/21
 * Code usage:
 * This is an instantiation file for "phases.cpp".
 * Any libraries used or function declarations are located here.
 */

#pragma once
#ifndef SETUP_H
#define SETUP_H

#define WAITTIME 30000

//define all of the pins used with particular names for identification //
#define ENCODER1APIN 2
#define ENCODER1BPIN 3
#define SEGCLK 4
#define SEGDIO 5
#define SERVOPIN 8
#define MOTOR_PIN 10
//PINS 11 & 12 CANNOT BE USED FOR PWM
#define LED_ON_BOARD 13
#define LIGHTBULBSWITCHPIN 15
#define PHONESWITCHPIN 16
#define ENCODER2APIN 18
#define ENCODER2BPIN 19
#define RESETSWITCHPIN 20
#define CONFIRMBUTTONPIN 21
#define P1RLED 22
#define P1GLED 23
#define P2RLED 24
#define P2GLED 25
#define P3RLED 26
#define P3GLED 27
#define P4RLED 28
#define P4GLED 29
#define DIRPIN 30
#define STEPPIN 31
#define ENPIN 32
#define HOMEPIN 33
#define AUDIOPIN 46
//50, 51, & 52 are used by the SD card reader
#define SDCSPIN 53
////////////////////////////////////////////////////////////////////////


//include all libraries used //
#include <Arduino.h>
#include <Encoder.h>  //https://github.com/PaulStoffregen/Encoder
#include <TM1637.h>   //https://github.com/AKJ7/TM1637
#include <SD.h>
#include <TMRpcm.h>   //https://www.arduino.cc/reference/en/libraries/tmrpcm/
#include <SPI.h>
///////////////////////////////


// instantiate important global variables //
extern volatile byte currentPhase;
extern volatile bool phaseChange;
extern volatile long long lastResponse;
////////////////////////////////////////////

// instantiate all functions used in program //
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
byte completion();
void sleep();
void error();
void reset();

void servoMove(uint16_t position);
void stepperTick();
void homeStepper();
void StepperSetup();
void resetPhases();
int8_t encoderRead(char enc);
void initSevenSegment();
void displayDigitalNumber(float value);
void setDCMotor(uint16_t pwmValue);
void fail_state_audio();
int mapValues(int x, int in_min, int in_max, int out_min, int out_max);
////////////////////////////////////////////////
#endif
