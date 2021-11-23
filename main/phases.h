/*
 * File name:         "phases.h"
 * Contributor(s):    Elliot Eickholtz, Matthew Wrocklage, Jackson Couch, Andrew Boehm
 * Last edit:         11/22/21
 * Code usage:
 * This is an instantiation file for "phases.cpp".
 * Any libraries used or function declarations are located here.
 */

#ifndef SETUP_H
#define SETUP_H

//PINS 11 & 12 CANNOT BE USED FOR PWM

#define servoPin 9 // Define the servo pin:

#define LED_ON_BOARD 13
#define MOTOR_PIN 10
#define SD_ChipSelectPin 4

#include <Arduino.h>
#include <Encoder.h>  //https://github.com/PaulStoffregen/Encoder
#include <TM1637.h>   //https://github.com/AKJ7/TM1637
#include <Servo.h>
#include <SD.h>
#include <TMRpcm.h>   //https://www.arduino.cc/reference/en/libraries/tmrpcm/
#include <SPI.h>

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

int8_t encoderRead(char enc);
void initSevenSegment();
void displayDigitalNumber(float value);
void setDCMotor(uint16_t pwmValue);
void fail_state_audio();

#endif
