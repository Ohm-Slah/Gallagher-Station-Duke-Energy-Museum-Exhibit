/*
 * File name:         "phases.h"
 * Contributor(s):    Elliot Eickholtz, Matthew Wrocklage, Jackson Couch, Andrew Boehm
 * Last edit:         2/18/22
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

//--------J23--------//
#define ENCODER1APIN 9
#define ENCODER1BPIN 19
// GND
#define ENCODER2APIN 10
#define ENCODER2BPIN 18
// GND
#define ENCODER3APIN 11
#define ENCODER3BPIN 2
// GND
#define ENCODER4APIN 12
#define ENCODER4BPIN 3
// GND
//^^^^^^^^^^^^^^^^^^^//

//--------J9---------//
#define MOTOR_PIN 23
#define DIRPIN 25
#define STEPPIN 27
#define ENPIN 22
#define HOMEPIN 24
// GND
// NC
//^^^^^^^^^^^^^^^^^^^//

//--------J1---------//
#define P1LEDPIN 21
#define P2LEDPIN 20
#define P3LEDPIN 17
#define P4LEDPIN 16
#define COALLEDPIN 15
#define AIRLEDPIN 14
#define VOLTAGELEDPIN 4
#define STEAMLEDPIN 5
#define CONFIRMBUTTONLEDPIN 6
#define SENDPOWERBUTTONLEDPIN 7
#define MAINSWITCHLEDPIN 8
//^^^^^^^^^^^^^^^^^^^//

//--------J6---------//
// 5V
#define TEMPERATURESERVOPIN 44
// GND
// 5V
#define VOLTAGESERVOPIN 45
// GND
// NC
//^^^^^^^^^^^^^^^^^^^//

//--------J41--------//
// 5V
// 0
// 1
// GND
// NC
//^^^^^^^^^^^^^^^^^^^//

//--------J48--------//
// 5V
// GND
//^^^^^^^^^^^^^^^^^^^//

//--------J15--------//
#define AUDIOPIN 46
// GND
//^^^^^^^^^^^^^^^^^^^//

//--------J35--------//
#define SENDPOWERBUTTONPIN 40
// GND
//^^^^^^^^^^^^^^^^^^^//

//--------J49--------//
#define PHONESWITCHPIN 16
// GND
//^^^^^^^^^^^^^^^^^^^//

//--------J29--------//
#define RESETSWITCHPIN 34
// GND
//^^^^^^^^^^^^^^^^^^^//

//--------J31--------//
#define LIGHTBULBSWITCHPIN 15
// GND
//^^^^^^^^^^^^^^^^^^^//

//--------J33--------//
#define CONFIRMBUTTONPIN 35
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
        File root;

        AudioPlaybackFromSDCard()
        {
            tmrpcm.speakerPin = AUDIOPIN;
            tmrpcm.disable();
            if (!SD.begin(SDCSPIN))
            {
                Serial.println("NO SD CARD");
                error();
            }
            root = SD.open("/");      // open SD card main root
            tmrpcm.setVolume(4);    //   0 to 7. Set volume level
            tmrpcm.quality(1);      //  Set 1 for 2x oversampling Set 0 for normal
        }

        void playFailureAudio()
        {
            if ( !tmrpcm.isPlaying() ) {
                // no audio file is playing
                File entry =  root.openNextFile();  // open next file
                if (! entry) {
                    // no more files
                    root.rewindDirectory();  // go to start of the folder
                    return;
                }

                uint8_t nameSize = String(entry.name()).length();  // get file name size
                String str1 = String(entry.name()).substring( nameSize - 4 );  // save the last 4 characters (file extension)

                if ( str1.equalsIgnoreCase(".wav") ) {
                    // the opened file has '.wav' extension
                    tmrpcm.play( entry.name() );      // play the audio file
                    Serial.print("Playing file: ");
                    Serial.println( entry.name() );
                }

                else {
                    // not '.wav' format file
                    entry.close();
                    return;
                }
            }
        }

        void disablePlayback()
        {
            tmrpcm.disable();
        }
};

#endif // SETUP_H
