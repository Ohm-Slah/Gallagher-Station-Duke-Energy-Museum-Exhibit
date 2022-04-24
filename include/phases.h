/*
 * File name:         "phases.h"
 * Contributor(s):    Elliot Eickholtz, Matthew Wrocklage, Jackson Couch, Andrew Boehm
 * Last edit:         4/21/22
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
//in order of junctions from the custom PCB                            //
//------------------------Start of Block-------------------------------//

//-------------------//
#define ENCODER1APIN 9      //air encoder A
#define ENCODER1BPIN 19     //air encoder B
// GND
#define ENCODER2APIN 10     //rheostat encoder A     
#define ENCODER2BPIN 18     //rheostat encoder B
// GND
#define ENCODER3APIN 11     //governor valve encoder A
#define ENCODER3BPIN 2      //governor valve encoder B
// GND
#define ENCODER4APIN 12     //coal encoder A
#define ENCODER4BPIN 43     //coal encoder B
// GND
//^^^^^^^^^^^^^^^^^^^//

//-------------------//
#define MOTOR_PIN 7
#define DIRPIN 25
#define STEPPIN 27
#define STEPPERENPIN 22	
#define HOMEPIN 24
// GND
#define DCMOTORENPIN 33
//^^^^^^^^^^^^^^^^^^^//

//-------------------//
#define P1LEDPIN 26
#define P2LEDPIN 28
#define P3LEDPIN 21
#define P4LEDPIN 20
#define COALLEDPIN 15
#define AIRLEDPIN 6
#define REOSTATLEDPIN 23
#define STEAMLEDPIN 5
#define CONFIRMBUTTONLEDPIN 4
#define SENDPOWERBUTTONLEDPIN 14
#define MAINSWITCHLEDPIN 8
//^^^^^^^^^^^^^^^^^^^//

//-------------------//
// 5V
#define TEMPERATURESERVOPIN 45
// GND
// 5V
#define AMPERAGESERVOPIN 44
// GND
// NC
//^^^^^^^^^^^^^^^^^^^//

//-------------------//
// 5V
// 20
// 21
// GND
// NC
//^^^^^^^^^^^^^^^^^^^//

//-------------------//
// 5V
// GND
//^^^^^^^^^^^^^^^^^^^//

//-------------------//
#define AUDIOPIN 46
// GND
//^^^^^^^^^^^^^^^^^^^//

//-------------------//
#define SENDPOWERBUTTONPIN 41
// GND
//^^^^^^^^^^^^^^^^^^^//

//-------------------//
#define PHONESWITCHPIN 39
// GND
//^^^^^^^^^^^^^^^^^^^//

//-------------------//
#define RESETSWITCHPIN 3
// GND
//^^^^^^^^^^^^^^^^^^^//

//-------------------//
#define LIGHTBULBSWITCHPIN 40
// GND
//^^^^^^^^^^^^^^^^^^^//

//-------------------//
#define CONFIRMBUTTONPIN 42
// GND
//^^^^^^^^^^^^^^^^^^^//

#define LED_ON_BOARD 13
#define SDCSPIN 53
// 50, 51, & 52 are used by the SD card reader library "SD.h"

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
#include "TMRpcm.h"   //https://github.com/TMRh20/TMRpcm
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

void intro();
bool serialResponse(char com[]);
bool serialWait();
void failure(uint8_t phase, uint8_t failureReason);
byte completion();
void sleep();
void error();
void reset();
void sendEvent(uint16_t data);

void printDirectory(File dir, int numTabs);
void initSDCard();
void playFailureAudio();
void disablePlayback();
void updateLEDS();
void phaseChangeLEDState(uint8_t phase);
void servoMove(uint16_t position);
void resetPhases();
int8_t encoderRead(char enc);
void setDCMotor(uint16_t pwmValue);
void disableDCMotor();
int mapValues(int x, int in_min, int in_max, int out_min, int out_max);
//^^^^^^^^^^^End of Block^^^^^^^^^^^^^^^//

// Class for the use of a stepper motor. This is technically not necessary,
// but it makes all code for the stepper motor code centralized and more readable.
class Stepper 
{
    public:
        Stepper()   //constructor, this is called when an instance is created
        {
            pinMode(STEPPERENPIN, OUTPUT);
            pinMode(STEPPIN, OUTPUT);
            pinMode(HOMEPIN, OUTPUT);
            pinMode(DIRPIN, OUTPUT);  
            disable();
        }

        void singleStep(bool direction) // iterate stepper position in direction given
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
            delayMicroseconds(10);   // allow enough clock cycles to set STEPPIN to 5V
            digitalWrite(STEPPIN, LOW);
        }

        void homeStepper()  // place needle on known position of dial to track position
        {
            digitalWrite(STEPPERENPIN, LOW);   // enable stepper motor

            // rotate stepper motor until the homing switch is triggered.
            while (digitalRead(HOMEPIN))
            {
                updateLEDS();
                singleStep(true);
                delay(8);   // change delay to change speed of motor
            }
            stepperPosition = 0;
        }

        void disable()  // disable stepper motor
        {
            digitalWrite(STEPPERENPIN, HIGH);
        }

        uint16_t stepperPosition; 
        const int stepsPerRevolution = 200;
};

class SevenSegmentDisplay
{
    public:
        SevenSegmentDisplay()
        {
            Serial2.begin(9600);
        }

        void display(char cStr[], int suffixMode)
        {
            //char cStr[7];  //buffer for float to str conversion
            char outStr[6] = {':',':',':',':',':',':'};
            byte offset = 0;

            //dtostrf(raw, 1, 1, cStr);   //raw -> cStr 1 dp of precision

            for(int i=0; i<7; i++)
            {
                switch(cStr[i])
                {
                    case '0': 
                        outStr[i-offset] = '0';
                    break;
                    case '1': 
                        outStr[i-offset] = '1';
                    break;
                    case '2': 
                        outStr[i-offset] = '2';
                    break;
                    case '3': 
                        outStr[i-offset] = '3';
                    break;
                    case '4':
                        outStr[i-offset] = '4';
                    break;
                    case '5': 
                        outStr[i-offset] = '5';
                    break;
                    case '6': 
                        outStr[i-offset] = '6';
                    break;
                    case '7': 
                        outStr[i-offset] = '7';
                    break;
                    case '8': 
                        outStr[i-offset] = '8';
                    break;
                    case '9': 
                        outStr[i-offset] = '9';
                    break;
                    case '.':
                        offset++;
                        // convert previous byte to DP equivalent
                        //Serial.print(cStr[i]);Serial.print(" : ");Serial.println(cStr[i-1]+11);
                        outStr[i-offset] = (cStr[i-1]+11);
                    break;
                    case 'A': 
                        outStr[i-offset] = 'E';
                    break;
                    case 'C': 
                        outStr[i-offset] = 'F';
                    break;
                    case 'H': 
                        outStr[i-offset] = 'G';
                    break;
                    case 'Z':
                        outStr[i-offset] = 'H';
                    break;
                    case ' ':
                        outStr[i-offset] = ':';
                    break;
                }
                //Serial.println(outStr);
                if(i-offset==5) break;
            }
            if (suffixMode==1)          // AC Suffix
            {
                outStr[4] = 'E';
                outStr[5] = 'F';
        
            } else if (suffixMode==2)   // HZ Suffix
            {
                outStr[4] = 'G';
                outStr[5] = 'H';
            }
            Serial2.println(outStr);
        }
};

#endif // SETUP_H
