/*
 * File name:         "phases.cpp"
 * Contributor(s):    Elliot Eickholtz, Matthew Wrocklage, Jackson Couch
 * Last edit:         11/29/21
 * Code usage:
 * This is a file containing all functions used in each of the five phases of the "main.ino" file.
 * 
 */

#include "phases.h"

// Create a variable to store the servo position:
int angle = 0;
volatile bool phaseChange = false;
volatile byte currentPhase = 0;

Encoder AirandVoltage(2, 3);        
Encoder Coal(18  , 19);       /* Creates an Encoder object, using 2 pins. Creates mulitple Encoder objects, where each uses its own 2 pins. The first pin should be capable of interrupts. 
                             * If both pins have interrupt capability, both will be used for best performance. 
                             * Encoder will also work in low performance polling mode if neither pin has interrupts. 
                             */

TM1637 tm(4, 5);            /* Library instantiation for 7-segment display
                             * Pin 4 -> DIO
                             * Pin 5 -> CLK
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
   * This fuction is run once on startup. 
   * This is to simply initialize everything needed.
   */
  pinMode(15, OUTPUT);
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
  
  TCCR1A = 0;
  TIMSK1 = (1 << OCIE1A);
  sei();
  Serial.begin(9600);

  delay(100);

  StepperSetup();
  
  if (!serialResponse("RESPOND")) error();
                                  //Audio 
  tmrpcm.speakerPin=46;            //5,6,11 or 46 on Mega, 9 on Uno, Nano, etc
                                //Complimentary Output or Dual Speakers:
                                //pinMode(10,OUTPUT); Pin pairs: 9,10 Mega: 5-2,6-7,11-12,46-45 
  Serial.begin(9600);
  if(!SD.begin(SD_ChipSelectPin))
  {
    Serial.println("SD fail");
    return;
  }
  
}

byte phaseZero() 
{
  /*
   * This fuction is the 'awaiting user input' phase. This will run for a maximum of three hours,
   * and will then enter a sleep state. The only difference in the sleep state is what the raspberry pi displays.
   * When exiting the sleep state, homing everthing like the initialization stage is needed.
   */
  
  if (!serialResponse("PHASE ZERO")) error();
  delay(1000);
  return 1;

}

byte phaseOne() 
{
  /*
   * This function is the first phase of the display.
   * 
   * Steps:
   * play intro vid
   * play phase 1 instruction vid
   * balance air and coal encoders until temp servo gauge is at specified position,
   * simultaneously waiting for confirm button to be pressed
   * if result on servo gauge is within eror margins, continue onto next phase.
   * if result is outside error margins, play fail video? three tries?
   */
   //digitalWrite(enPin, LOW);
   int prevPos = 0;
   //tmrpcm.loop(1);
   //fail_state_audio();
//   ledBlink(255, 1000);
//   long i = 0;
//   while(1)
//   {
//    servoMove((int)map(i%255, 0, 255, 255, 0));
//    Serial.println((int)map(i%255, 0, 255, 255, 0));
//    if(i%255==0)
//    {
//      servoMove(0);
//      delay(1000);
//    }
//    i++;
//    delay(50);
//    digitalWrite(19, !digitalRead(21));
//    setDCMotor(!digitalRead(21)*100);
//    displayDigitalNumber(encoderRead('A'));
//    prevPos = encoderRead('C');
//   }

  //Make sure everything is at an off state while the intro plays 
  //if (!serialResponse("INTRO")) error();

  //homeStepper();
  servoMove(0);
  AirandVoltage.write(0);
  Coal.write(0);
  ledBlink(0, 1000);
  digitalWrite(19, LOW);

  //initialize temporary variables
  int8_t coalRead;
  int8_t airRead;
  int16_t coalAngle;
  int16_t airAngle;
  float airLine;
  uint16_t coalLine;
  uint16_t bottomLine = 1000;
  float tempLine; 
  //optimum temp of boiler: 2150 degF

  bool dir = false;  //0=left & 1=right
  uint8_t count = 0;
  
  while(digitalRead(21))
  {
    coalRead = encoderRead('C');
    airRead = encoderRead('A');

    if(count>abs(airAngle-coalAngle)*50) dir = !dir;

    bottomLine = 1000 - count;

    if(dir)
      count++;
    else
      count--;
      
    if(coalRead)
    {
      coalAngle += coalRead;
      Coal.write(0);
      if(coalAngle > 70)
      {
        coalAngle = 70;
      }
      else if(coalAngle < 0)
      {
        coalAngle = 0;
      }
    }
    if(airRead)
    {
      airAngle += airRead;
      AirandVoltage.write(0);
      if(airAngle > 70)
        airAngle = 70;
      else if(airAngle < 0)
        airAngle = 0;
    }
    
    airLine = ((float)bottomLine*sin((float)coalAngle*PI/180))/sin(((float)180-coalAngle-airAngle)*PI/180);
    tempLine = sin((float)airAngle*PI/180)*airLine;
    //Serial.println(tempLine, 4);
    servoMove(map((int)tempLine, 0, 1374, 255, 0));
    delay(5);
  }
  
  /* ADD SERIAL RESPONSE WAIT UNTIL END OF INTO VID */
  if(phaseChange) return 1;
  if (!serialResponse("PHASE ONE")) error();
  if(phaseChange) return 1;
  
  return 2;
}

byte phaseTwo() 
{
  /*
   * This function is the second phase of the display.
   */
  if (!serialResponse("PHASE TWO")) error();

  if(phaseChange) return 1;
  delay(1000);
  if(phaseChange) return 1;
  return 3;
}

byte phaseThree() 
{
  /*
   * This function is the third phase of the display.
   */
  if (!serialResponse("PHASE THREE")) error();
  if(phaseChange) return 1;
  delay(1000);
  if(phaseChange) return 1;
  return 4;
}

byte phaseFour() 
{
  /*
   * This function is the fourth phase of the display.
   */
  if (!serialResponse("PHASE FOUR")) error();
  if(phaseChange) return 1;
  delay(1000);
  if(phaseChange) return 1;
  return 10;
}

bool serialResponse(char com[])
{
  /*
   * This function takes a predefined string command and confirms a serial response from the raspberry pi running processing.
   * Predefined commands: "RESPOND" "PHASE ZERO" "PHASE ONE" "PHASE TWO" "PHASE THREE" "PHASE FOUR" "FAILURE" "COMPLETE"
   */
//  uint8_t attempts = 0;
// 
//  delay(100);
//  while(attempts <= 5)
//  {
//    Serial.println(com);
//    if (Serial.available())
//    {
//      char val = Serial.read();
//      Serial.println(val);
//      if (val == '1')
//      {
//        return true;
//      } else {
//        return false;
//        attempts++;
//      }
//    }
//    delay(100);
//    attempts++;
//  }
//  return false;
  return true;
  
}

void failure() 
{
  /*
   * This function is the failure state of the display.
   */
  if (!serialResponse("FAILURE")) error();
  delay(1000);
  return true;
}

void completion() 
{
  /*
   * This function is the completion state of the display.
   */
  if (!serialResponse("COMPLETE")) error();
  delay(1000);
  return true;
}

void error()
{
  /*
   * This function is the error state of the display. Only call this if a reset is necessary.
   */
  Serial.println("CRITICAL ERROR");
  while(1)
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
   * This function is attached to an interrupt, and resets any progress in the phases, bringing you back you phase 1.
   */
   phaseChange = true;
   currentPhase = 1;
   test();
}

void test()
{
  Serial.print("INTERRUPT : ");
  Serial.print(currentPhase);
  Serial.print(" : ");
  Serial.println(phaseChange);
}

void servoMove(uint16_t position)
{
  /*
   * This function recieves a position value and moves the servo to that position. 0-1023
   */
  analogWrite(servoPin, position);
}

int8_t encoderRead(char enc) 
{
  /*
   * This function takes in a character representing what encoder value you want returned. That value is then returned.
   */
  if (enc == 'A')                  //if Air Control
  {
    return AirandVoltage.read();  //returns the accumlated position (new position)
  } else if (enc == 'C')           //if Coal Control
  {
    return Coal.read();           //returns the accumlated position (new position)
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
   * This function runs once at startup and initializes the 7-segment display.
   */
  tm.begin();
  tm.setBrightness(4);
}

void displayDigitalNumber(float value)
{
  /*
   * This function takes in a 4-digit value, integer or float, and displays it on the 7-segment display.
   * Due to it's simplicity, it may be removed at a later date.
   */

  tm.display(value);

}

void setDCMotor(uint16_t pwmValue)
{
  /*
   * This fuction recieves an integer value and runs the DC motor at that PWM at 1024 precision.
   */
  analogWrite(MOTOR_PIN, pwmValue);
}



void ledStateChange(byte State)
{
  /*
   * This function takes in a byte (pins 22-29) representing the wanted state of the LEDs.
   */
   for(int i=22; i<30; i++)
   {
      digitalWrite(i, ~(State&(0x01<<(i-22) ) ) );
   }
}

void ledBlink(byte LED, int Time)
{
  /*
   * This function takes in a byte to select which pin to blink, and a int for how long to blink in ms.
   * The delay can be 500ms, 1000ms, 2000ms, or 4000ms.
   */
   ledState = LED;
   switch(Time)
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
  
  while(!digitalRead(homePin))
  {
    stepperTick();
    delay(20);
  }
  stepperPosition = 0;
}

void stepperTick()
{
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(stepPin, LOW);
  stepperPosition = (stepperPosition%stepsPerRevolution) + 1;
}


ISR(TIMER1_COMPA_vect)
{
  if (ledCount%2)
    ledStateChange(ledState);
  else
    ledStateChange(0);
    
  ledCount++;

}
  
void fail_state_audio()
{
  tmrpcm.setVolume(6);
  tmrpcm.play("JA.wav");
  delay(5000); 
}

//int mapValues(int x, int in_min, int in_max, int out_min, int out_max) {
//  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//}
