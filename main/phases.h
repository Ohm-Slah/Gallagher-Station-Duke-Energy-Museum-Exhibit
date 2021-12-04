/*
 * File name:         "phases.h"
 * Contributor(s):    Elliot Eickholtz, Matthew Wrocklage, Jackson Couch, Andrew Boehm
 * Last edit:         11/29/21
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
#define SD_ChipSelectPin 53   // example uses hardware SS pin 53 on Mega2560 or 10 for the UNO 

#include <Arduino.h>
#include <Encoder.h>  //https://github.com/PaulStoffregen/Encoder
#include <TM1637.h>   //https://github.com/AKJ7/TM1637
#include <SD.h>
#include <TMRpcm.h>   //https://www.arduino.cc/reference/en/libraries/tmrpcm/
#include <SPI.h>

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

void test();
#endif
