/*
  *File name: arduino_sd.ino
  *Contributors: Andrew Boehm
  *This program plays the fail state audio to the red phone
  *Must install TMRpcm library found here https://www.arduino.cc/reference/en/libraries/tmrpcm/ (click on read documentation and then go down to installation)
  *audio file should be a .wav file and must be on sd card
  *name of audio file must be inserted in the code below
*/
#include "SD.h"
#define SD_ChipSelectPin 4
#include "TMRpcm.h"
#include "SPI.h"

void setup(){ }

void fail_state_audio()
{
  TMRpcm tmrpcm;

  tmrpcm.speakerPin = 5;  //speaker output pin - can be 5,6,11, or 46
  Serial.begin(9600);
  if (!SD.begin(SD_ChipSelectPin)) 
  {
  Serial.println("SD fail");
  return;
  }

  tmrpcm.setVolume(5); //sets volume (0-7)
  tmrpcm.play("fitnessgram.wav"); //***insert file name here***

}

void loop() { }
