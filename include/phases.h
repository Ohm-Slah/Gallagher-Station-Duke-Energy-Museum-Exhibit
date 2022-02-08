/*
 * File name:         "phases.h"
 * Contributor(s):    Elliot Eickholtz, Matthew Wrocklage, Jackson Couch, Andrew Boehm
 * Last edit:         2/8/22
 * Code usage:
 * This is a header file for "phases.cpp".
 * All libraries used or function declarations are located and defined here.
 */

// These prevent the linker from including "phases.h" more than once
#pragma once
#ifndef SETUP_H
#define SETUP_H

// Time in milliseconds to wait before timeout to phase zero and to sleep mode respectively
#define WAITTIME 30000
#define SLEEPTIME 10800000


//define all of the pins used with particular names for identification //
//------------------------Start of Block-------------------------------//

// 0 & 1 are connected to ATTiny 4313 for UART communication for 7-segment display.
#define ENCODER1APIN 2
#define ENCODER1BPIN 3
// 4 & 5 are NC
#define CONFIRMBUTTONLEDPIN 6
#define SENDPOWERBUTTONLEDPIN 7
#define TEMPERATURESERVOPIN 8
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
#define SENDPOWERBUTTONPIN 40
#define VOLTAGESERVOPIN 41
// 42-45 are NC
#define AUDIOPIN 46
// 47-49 are NC
// 50, 51, & 52 are used by the SD card reader library "SD.h"
#define SDCSPIN 53
//^^^^^^^^^^^^^^^^^^^^^^^^End of Block^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^//


// include all libraries used //
//------Start of Block--------//

// Libraries with external links are imported.
// Libraries with <> are internal, 
// Libraries with "" are in the same directory as this file

// Standard Arduino library for I/O functionality
#include <Arduino.h>
// Library to count rotary encoder 'pulses' in the backgroud with interrupts
#include <Encoder.h>  //https://github.com/PaulStoffregen/Encoder
// Standard library of Arduino that allows SD card file manipulation
#include <SD.h>
// Library that links with SD.h, allows for audio playback through I/O
#include <TMRpcm.h>   //https://github.com/TMRh20/TMRpcm
// Standard Arduino SPI library
#include <SPI.h>
// Library that allows controlling the blinking of multiple LEDs with minimal interaction
#include "TimedBlink.h"

//^^^^^^End of Block^^^^^^^^^^//


// instantiate important global variables //
//-----------Start of Block---------------//
extern volatile byte currentPhase;
extern volatile bool phaseChange;
extern volatile long long lastResponse;
//^^^^^^^^^^^End of Block^^^^^^^^^^^^^^^^^//


// define all functions used in program //
//-----------Start of Block-------------//
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
void resetPhases();
int8_t encoderRead(char enc);
void setDCMotor(uint16_t pwmValue);
int mapValues(int x, int in_min, int in_max, int out_min, int out_max);
//^^^^^^^^^^^End of Block^^^^^^^^^^^^^^^//


// Class for the use of a stepper motor. This is technically not necessary,
// but it makes all code for the stepper motor centralized and more readable.
class Stepper 
{
    public:
        Stepper()   //constructor, this is called when an instance is created
        {
            pinMode(ENPIN, OUTPUT);
            pinMode(STEPPIN, OUTPUT);
            pinMode(HOMEPIN, OUTPUT);
            pinMode(DIRPIN, OUTPUT);  
            disable();
        }

        void singleStep(bool direction)
        {
            if(direction) // true = CW / false = CCW
            { 
                digitalWrite(DIRPIN, HIGH);
                // keep track of the stepper motor position
                stepperPosition = (stepperPosition + 1) % stepsPerRevolution;
            } else 
            {
                digitalWrite(DIRPIN, LOW);
                // keep track of the stepper motor position
                stepperPosition = (stepperPosition + stepsPerRevolution - 1) % stepsPerRevolution;
            }

            digitalWrite(STEPPIN, HIGH);
            delayMicroseconds(10);   //allow enough clock cycles to set STEPPIN to 5V
            digitalWrite(STEPPIN, LOW);
        }

        void homeStepper()
        {
            digitalWrite(ENPIN, LOW);

            // rotate stepper motor until the homing switch is triggered.
            while (!digitalRead(HOMEPIN))
            {
                singleStep(true);
                delay(4);
            }
            stepperPosition = 0;
        }

        void disable()
        {
            digitalWrite(ENPIN, HIGH);
        }

        uint16_t stepperPosition; 
        const int stepsPerRevolution = 200;
};


// Class for the use of the phone speaker. This is technically not necessary,
// but it makes all code for the phone centralized and more readable.
class AudioPlaybackFromSDCard
{
    public:
        TMRpcm tmrpcm; //create instance for sd card reading

        AudioPlaybackFromSDCard()
        {
            tmrpcm.speakerPin = AUDIOPIN;
            tmrpcm.disable();
        }

        void playFailureAudio()
        {
            tmrpcm.setVolume(6);
            tmrpcm.play("JA.wav");
        }

        void disablePlayback()
        {
            tmrpcm.disable();
        }
};

#endif // SETUP_H
