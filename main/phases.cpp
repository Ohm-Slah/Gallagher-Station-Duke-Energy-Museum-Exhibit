/*
   File name:         "phases.cpp"
   Contributor(s):    Elliot Eickholtz, Matthew Wrocklage, Jackson Couch, Andrew Boehm
   Last edit:         12/4/21
   Code usage:
   This is a file containing all functions used in each of the five phases of the "main.ino" file.

*/

#include "phases.h"

// Create a variable to store the servo position:
int angle = 0;
volatile bool phaseChange = false;
volatile byte currentPhase = 0;

Encoder AirandVoltage(2, 3);
Encoder CoalandSteam(18, 19);       /* Creates an Encoder object, using 2 pins. Creates mulitple Encoder objects, where each uses its own 2 pins. The first pin should be capable of interrupts.
                               If both pins have interrupt capability, both will be used for best performance.
                               Encoder will also work in low performance polling mode if neither pin has interrupts.
*/

TM1637 tm(4, 5);            /* Library instantiation for 7-segment display
                               Pin 4 -> DIO
                               Pin 5 -> CLK
*/

const uint16_t halfTwoSec = 31250;
const uint16_t fullFourSec = 62500;
volatile byte ledState = 0;
volatile long ledCount = 0;

//integers for the stepper motor
const int dirPin = 30;
const int stepPin = 31;
const int enPin = 32;
const int homePin = 33;
const int stepsPerRevolution = 200;

long stepperPosition;

TMRpcm tmrpcm; //This is for the audio

bool zeroPassed = false;
bool onePassed = false;
bool twoPassed = false;
bool threePassed = false;
bool fourPassed = false;

void initialization()
{
  /*
     This fuction is run once on startup.
     This is to simply initialize everything needed.
  */
  pinMode(16, INPUT_PULLUP); //Phone Switch
  pinMode(15, OUTPUT); //Light bulb
  pinMode(30, INPUT); //Confirm button
  pinMode(31, INPUT); //Send Power button
  pinMode(22, OUTPUT); //Phase 1 Red LED
  pinMode(23, OUTPUT); //Phase 1 Green LED
  pinMode(24, OUTPUT); //Phase 2 Red LED
  pinMode(25, OUTPUT); //Phase 2 Green LED
  pinMode(26, OUTPUT); //Phase 3 Red LED
  pinMode(27, OUTPUT); //Phase 3 Green LED
  pinMode(28, OUTPUT); //Phase 4 Red LED
  pinMode(29, OUTPUT); //Phase 4 Green LED
  pinMode(LED_ON_BOARD, OUTPUT); //LED pin of Arduino Mega
  pinMode(MOTOR_PIN, OUTPUT); //DC Motor Pin
  pinMode(servoPin, OUTPUT); //Servo motor Pin
  pinMode(20, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(20), resetPhases, FALLING);
  pinMode(21, INPUT_PULLUP);

  initSevenSegment();

  TCCR1A = 0;
  TIMSK1 = (1 << OCIE1A);
  sei();
  Serial.begin(9600);

  delay(100);

  StepperSetup();

  if (!serialResponse("RESPOND")) error();
  //Audio
  tmrpcm.speakerPin = 46;          //5,6,11 or 46 on Mega, 9 on Uno, Nano, etc
  //Complimentary Output or Dual Speakers:
  //pinMode(10,OUTPUT); Pin pairs: 9,10 Mega: 5-2,6-7,11-12,46-45
  Serial.begin(9600);
  if (!SD.begin(SD_ChipSelectPin))
  {
    Serial.println("SD fail");
    return;
  }

}

void reset()
{
  homeStepper();

  AirandVoltage.write(0);
  CoalandSteam.write(0);
  ledBlink(0, 1000);
  digitalWrite(23, HIGH);
  digitalWrite(25, HIGH);
  digitalWrite(27, HIGH);
  digitalWrite(29, HIGH);
  digitalWrite(15, LOW);
  servoMove(1);
  setDCMotor(0);
  tm.clearScreen();
}

byte phaseZero()
{
  /*
     This fuction is the 'awaiting user input' phase. This will run for a maximum of three hours,
     and will then enter a sleep state. The only difference in the sleep state is what the raspberry pi displays.
     When exiting the sleep state, homing everthing like the initialization stage is needed.
  */

  if (!serialResponse("PHASE ZERO")) error();
  delay(1000);
  return 1;

}

byte phaseOne()
{
  /*
     This function is the first phase of the display.

     Steps:
     play intro vid
     play phase 1 instruction vid
     balance air and coal encoders until temp servo gauge is at specified position,
     simultaneously waiting for confirm button to be pressed
     if result on servo gauge is within eror margins, continue onto next phase.
     if result is outside error margins, play fail video? three tries?
  */
  int prevPos = 0;
  //tmrpcm.loop(1);
  //fail_state_audio();

  //Make sure everything is at an off state while the intro plays
  reset();

  //initialize temporary variables
  int8_t coalRead = 0;
  int8_t airRead = 0;
  int16_t coalAngle = 0;
  int16_t airAngle = 0;
  float airLine = 0;
  uint16_t coalLine = 0;
  uint16_t bottomLine = 1000;
  float tempLine = 0;
  //optimum temp of boiler: 2150 degF

  bool dir = false;  //0=left & 1=right
  uint8_t count = 0;

  /* ADD SERIAL RESPONSE WAIT UNTIL END OF INTO VID */
  if (!serialResponse("PHASE ONE")) {
    error();
  }

  ledBlink(0B00000001, 1000);

  while (digitalRead(21))
  {
    if (phaseChange) return 1;
    coalRead = encoderRead('C');
    airRead = encoderRead('A');

    if (count > abs(airAngle - coalAngle) * 50) dir = !dir;

    bottomLine = 1000 - count;

    if (dir)
      count++;
    else
      count--;

    if (coalRead)
    {
      coalAngle += coalRead;
      CoalandSteam.write(0);
      if (coalAngle > 70)
      {
        coalAngle = 70;
      }
      else if (coalAngle < 0)
      {
        coalAngle = 0;
      }
    }
    if (airRead)
    {
      airAngle += airRead;
      AirandVoltage.write(0);
      if (airAngle > 70)
        airAngle = 70;
      else if (airAngle < 0)
        airAngle = 0;
    }

    airLine = ((float)bottomLine * sin((float)coalAngle * PI / 180)) / sin(((float)180 - coalAngle - airAngle) * PI / 180);
    tempLine = sin((float)airAngle * PI / 180) * airLine;
    //Serial.print(map((int)tempLine, 0, 1374, 255, 0));Serial.print(" : ");Serial.print(airAngle);Serial.print(" : ");Serial.println(coalAngle);
    servoMove(map((int)tempLine, 0, 1374, 255, 0));
    delay(5);
  }

  if (abs(tempLine - 926) < 50 && abs(airAngle - coalAngle) < 5)
  {
    //Serial.println("SUCCESS");
    digitalWrite(23, LOW);
    servoMove(79);
    return 2;
  } else {
    failure();
    return 1;
  }

}

byte phaseTwo()
{
  /*
     This function is the second phase of the display.
  */
  delay(1000);
  if (!serialResponse("PHASE TWO")) error();
  CoalandSteam.write(0);
  tm.colonOff();
  int16_t steamRead = 0;
  int16_t steam = 0;
  ledBlink(0B00000101, 1000);

  while (digitalRead(21))
  {
    if (phaseChange) return 1;

    if (steamRead)
    {
      steam += steamRead;
      CoalandSteam.write(0);
      if (steam > 255)
      {
        steam = 255;
      }
      else if (steam < 0)
      {
        steam = 0;
      }
      setDCMotor(steam);
      tm.clearScreen();
      tm.display(steam, true, false, 0);
    }


    steamRead = encoderRead('C');
  }

  digitalWrite(25, LOW);
  if (phaseChange) return 1;
  return 3;
}

byte phaseThree()
{
  /*
     This function is the third phase of the display.
     It is currently is in use for a demonstration.
  */
  if (!serialResponse("PHASE THREE")) error();
  ledBlink(0B00010101, 1000);
  delay(1000);
  CoalandSteam.write(0);
  int16_t steamRead = 0;
  int16_t steam = 0;
  int16_t steamPrev = 0;

  while (digitalRead(21))
  {
    if (phaseChange) return 1;
    steamRead = encoderRead('C');
    if (steamRead)
    {
      steam += steamRead;
      CoalandSteam.write(0);
      if (steam > steamPrev)
      {
        digitalWrite(dirPin, LOW);
        for (int i = steamPrev; i < steam; i++)
        {
          stepperTick();
          delay(5);
        }
        steamPrev = steam;
      } else if (steam < steamPrev)
      {
        digitalWrite(dirPin, HIGH);
        for (int i = steamPrev; i > steam; i--)
        {
          stepperTick();
          delay(5);
        }
        steamPrev = steam;
      }
    }

  }
  digitalWrite(27, LOW);
  digitalWrite(29, LOW);
  digitalWrite(enPin, HIGH);
  digitalWrite(15, HIGH);
  return 10; //temporarily skip phase four
}

byte phaseFour()
{
  /*
     This function is the fourth phase of the display.
  */
  if (!serialResponse("PHASE FOUR")) error();

  while (digitalRead(21))
  {
    if (phaseChange) return 1;
  }
  return 10;
}

bool serialResponse(char com[])
{
  /*
     This function takes a predefined string command and confirms a serial response from the raspberry pi running processing.
     Predefined commands: "RESPOND" "RING" "PHASE ZERO" "PHASE ONE" "PHASE TWO" "PHASE THREE" "PHASE FOUR" "FAILURE" "COMPLETE"
  */
  uint8_t attempts = 0;

  delay(1000);
  while (attempts <= 5)
  {
    Serial.println(com);
    if (Serial.available())
    {
      char val = Serial.read();
      //Serial.println(val);
      if (val == '1')
      {
        return true;
      } else {
        return false;
        attempts++;
      }
    }
    delay(2000);
    attempts++;
  }
  return false;

}

void failure()
{
  /*
     This function is the failure state of the display.
  */
  delay(1000);
  if (!serialResponse("RING")) error();
  while(!digitalRead(16)) {};
  if (!serialResponse("FAILURE")) error();
  //tmrpcm.disable();
  fail_state_audio();
  delay(5000);
  tmrpcm.disable();
  
  while(digitalRead(16)) {};
  return true;
}

void completion()
{
  /*
     This function is the completion state of the display.
  */
  if (!serialResponse("COMPLETE")) error();
  delay(5000);
  return true;
}

void error()
{
  /*
     This function is the error state of the display. Only call this if a reset is necessary.
  */
  Serial.println("CRITICAL ERROR");
  while (1)
  {
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
  }
}

void resetPhases()
{
  /*
     This function is attached to an interrupt, and resets any progress in the phases, bringing you back you phase 1.
  */
  phaseChange = true;
  currentPhase = 1;
}

void servoMove(uint16_t position)
{
  /*
     This function recieves a position value and moves the servo to that position. 0-255
  */
  analogWrite(servoPin, position);
}

int8_t encoderRead(char enc)
{
  /*
     This function takes in a character representing what encoder value you want returned. That value is then returned.
  */
  if (enc == 'A')                  //if Air Control
  {
    return AirandVoltage.read();  //returns the accumlated position (new position)
  } else if (enc == 'C')           //if Coal Control
  {
    return CoalandSteam.read();           //returns the accumlated position (new position)
  } else if (enc == 'V')           //if Voltage Control
  {
    return AirandVoltage.read();  //returns the accumlated position (new position)
  } else
  {
    return;
  }


}

void initSevenSegment()
{
  /*
     This function runs once at startup and initializes the 7-segment display.
  */
  tm.begin();
  tm.setBrightness(4);
}

void displayDigitalNumber(float value)
{
  /*
     This function takes in a 4-digit value, integer or float, and displays it on the 7-segment display.
     Due to it's simplicity, it may be removed at a later date.
  */

  tm.display(value);

}

void setDCMotor(uint16_t pwmValue)
{
  /*
     This fuction recieves an integer value and runs the DC motor at that PWM at 1024 precision.
  */
  analogWrite(MOTOR_PIN, pwmValue);
}



void ledStateChange(byte State)
{
  /*
     This function takes in a byte (pins 22-29) representing the wanted state of the LEDs.
  */
//  for (int i = 22; i < 30; i++)
//  {
//    digitalWrite(i, ~(ledState & (0x01 << (i - 22) ) ) );
//  }
   digitalWrite(22, !(State&0b00000001));
   //digitalWrite(23, !(State&0b00000010));
   digitalWrite(24, !(State&0b00000100));
   //digitalWrite(25, !(State&0b00001000));
   digitalWrite(26, !(State&0b00010000));
   //digitalWrite(27, !(State&0b00100000));
   digitalWrite(28, !(State&0b01000000));
   //digitalWrite(29, !(State&0b10000000));
   //Serial.println(State);
}

void ledBlink(byte LED, int Time)
{
  /*
     This function takes in a byte to select which pin to blink, and a int for how long to blink in ms.
     The delay can be 500ms, 1000ms, 2000ms, or 4000ms.
  */
  ledState = LED;
  switch (Time)
  {
    case 500:
      TCCR1B |= (1 << CS12);
      TCCR1B &= ~(1 << CS11);
      TCCR1B &= ~(1 << CS10);
      TCNT1 = 0;
      OCR1A = halfTwoSec;
      break;
    case 1000:
      TCCR1B |= (1 << CS12);
      TCCR1B &= ~(1 << CS11);
      TCCR1B &= ~(1 << CS10);
      TCNT1 = 0;
      OCR1A = fullFourSec;
      break;
    case 2000:
      TCCR1B |= (1 << CS12);
      TCCR1B &= ~(1 << CS11);
      TCCR1B |= (1 << CS10);
      TCNT1 = 0;
      OCR1A = halfTwoSec;
      break;
    case 4000:
      TCCR1B |= (1 << CS12);
      TCCR1B &= ~(1 << CS11);
      TCCR1B |= (1 << CS10);
      TCNT1 = 0;
      OCR1A = fullFourSec;
      break;
  }
}

void StepperSetup()
{
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  pinMode(homePin, INPUT);

  // Set motor direction clockwise
  digitalWrite(dirPin, HIGH);
  // disable stepper
  digitalWrite(enPin, HIGH);
}

void homeStepper()
{
  digitalWrite(enPin, LOW);

  while (!digitalRead(homePin))
  {
    stepperTick();
    delay(4);
  }
  stepperPosition = 0;
}

void stepperTick()
{
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(stepPin, LOW);
  stepperPosition = (stepperPosition % stepsPerRevolution) + 1;
}


ISR(TIMER1_COMPA_vect)
{
  if (ledCount % 2)
    ledStateChange(0);
  else
    ledStateChange(ledState);

  ledCount++;

}

void fail_state_audio()
{
  tmrpcm.setVolume(6);
  tmrpcm.play("JA.wav");
  //delay(5000);
}

//int mapValues(int x, int in_min, int in_max, int out_min, int out_max) {
//  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//}
