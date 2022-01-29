/*
 * File name:         "phases.h"
 * Contributor(s):    Elliot Eickholtz, Matthew Wrocklage, Jackson Couch, Andrew Boehm
 * Last edit:         1/29/22
 * Code usage:
 * This is an instantiation file for "phases.cpp".
 * Any libraries used or function declarations are located here.
 */

#pragma once
#ifndef SETUP_H
#define SETUP_H

#define WAITTIME 30000
#define SLEEPTIME 10800000

//---------------------------------------------------------------------//
//define all of the pins used with particular names for identification //
#define ENCODER1APIN 2
#define ENCODER1BPIN 3
#define SEGCLKPIN 4
#define SEGDIOPIN 5
#define CONFIRMBUTTONLEDPIN 6
#define SENDPOWERBUTTONLEDPIN 7
#define SERVOPIN 8
#define MAINSWITCHLEDPIN 9
#define MOTOR_PIN 10
// 11 & 12 are NC
#define LED_ON_BOARD 13
#define LIGHTBULBSWITCHPIN 15
#define PHONESWITCHPIN 16
#define ENCODER2APIN 18
#define ENCODER2BPIN 19
// I2C communication uses 20 & 21
#define P1LEDPIN 22
#define P2LEDPIN 23
#define P3LEDPIN 24
#define P4LEDPIN 25
#define COALLEDPIN 26
#define AIRLEDPIN 27
#define VOLTAGELEDPIN 28
#define STEAMLEDPIN 29
#define DIRPIN 30
#define STEPPIN 31
#define ENPIN 32
#define HOMEPIN 33
#define RESETSWITCHPIN 34
#define CONFIRMBUTTONPIN 35
#define ENCODER3APIN 36
#define ENCODER3BPIN 37
#define ENCODER4APIN 38
#define ENCODER4BPIN 39
// 40-45 are NC
#define AUDIOPIN 46
// 47-49 are NC
// 50, 51, & 52 are used by the SD card reader
#define SDCSPIN 53
//---------------------------------------------------------------------//

//include all libraries used //
#include <Arduino.h>
#include <Encoder.h>  //https://github.com/PaulStoffregen/Encoder
#include <TM1637.h>   //https://github.com/AKJ7/TM1637
#include <SD.h>
#include <TMRpcm.h>   //https://github.com/TMRh20/TMRpcm
#include <SPI.h>
#include "TimedBlink.h"
//---------------------------------------------------------------------//

// instantiate important global variables //
extern volatile byte currentPhase;
extern volatile bool phaseChange;
extern volatile long long lastResponse;
//---------------------------------------------------------------------//

// instantiate all functions used in program //
void initialization();
byte phaseZero();
byte phaseOne();
byte phaseTwo();
byte phaseThree();
byte phaseFour();

bool serialResponse(char com[]);
bool serialWait();
void failure();
byte completion();
void sleep();
void error();
void reset();

void updateLEDS();
void phaseChangeLEDState(uint8_t phase);
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

#endif
//---------------------------------------------------------------------//