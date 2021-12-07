/*
   File name:         "phases.cpp"
   Contributor(s):    Elliot Eickholtz, Matthew Wrocklage, Jackson Couch, Andrew Boehm
   Last edit:         12/6/21
   Code usage:
   This is a file containing all functions used in each of the five phases of the "main.ino" file.
*/

#include "phases.h"

// Create a variable to store the servo position
int angle = 0;

// reintantiate global use variables
volatile bool phaseChange = false;
volatile byte currentPhase = 0;

// Creates an Encoder object, using 2 pins.
Encoder AirandVoltage(ENCODER1APIN, ENCODER1BPIN); 
Encoder CoalandSteam(ENCODER2APIN, ENCODER2BPIN); 

// Library instantiation for 7-segment display
TM1637 tm(SEGCLK, SEGDIO);  

// This is for the audio
TMRpcm tmrpcm; 
                               
// intantiate variables for led blinking interrupts
const uint16_t halfTwoSec = 31250;
const uint16_t fullFourSec = 62500;
volatile byte ledState = 0;
volatile long ledCount = 0;

// variables for the stepper motor
const int stepsPerRevolution = 200;
long stepperPosition;

void initialization()
{
  /*
     This fuction is run once on startup.
     This is to simply initialize everything needed.
  */
  pinMode(PHONESWITCHPIN, INPUT_PULLUP); //Phone Switch
  pinMode(LIGHTBULBSWITCHPIN, OUTPUT); //Light bulb
  pinMode(31, INPUT); //Send Power button
  pinMode(P1RLED, OUTPUT); //Phase 1 Red LED
  pinMode(P1GLED, OUTPUT); //Phase 1 Green LED
  pinMode(P2RLED, OUTPUT); //Phase 2 Red LED
  pinMode(P2GLED, OUTPUT); //Phase 2 Green LED
  pinMode(P3RLED, OUTPUT); //Phase 3 Red LED
  pinMode(P3GLED, OUTPUT); //Phase 3 Green LED
  pinMode(P4RLED, OUTPUT); //Phase 4 Red LED
  pinMode(P4GLED, OUTPUT); //Phase 4 Green LED
  pinMode(LED_ON_BOARD, OUTPUT); //LED pin of Arduino Mega
  pinMode(MOTOR_PIN, OUTPUT); //DC Motor Pin
  pinMode(SERVOPIN, OUTPUT); //Servo motor Pin
  pinMode(RESETSWITCHPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RESETSWITCHPIN), resetPhases, FALLING);
  pinMode(CONFIRMBUTTONPIN, INPUT_PULLUP);

  

  TCCR1A = 0;
  TIMSK1 = (1 << OCIE1A);
  sei();
  Serial.begin(9600);
  delay(100);

  StepperSetup();
  initSevenSegment();
  tmrpcm.speakerPin = SDCSPIN;

  if (!SD.begin(SDCSPIN))
    error();

  if (!serialResponse("RESPOND")) error();

  

}

void reset()
{
  /*
   * This function can be ran when all items on board need to be reset
   */
  homeStepper();

  AirandVoltage.write(0);
  CoalandSteam.write(0);
  ledBlink(0, 1000);
  digitalWrite(P1GLED, HIGH);
  digitalWrite(P2GLED, HIGH);
  digitalWrite(P3GLED, HIGH);
  digitalWrite(P4GLED, HIGH);
  digitalWrite(LIGHTBULBSWITCHPIN, LOW);
  servoMove(1);
  setDCMotor(0);
  tm.clearScreen();
}

byte phaseZero()
{
  /*
     This fuction is the 'awaiting user input' phase. This will run for a maximum of X hours,
     and will then enter a sleep state. The only difference in the sleep state is what the raspberry pi displays.
     When exiting the sleep state, reseting function is needed.
     This currently does nothing.
  */

  if (!serialResponse("PHASE ZERO")) error();
  
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
     if result is outside error margins, play fail video
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

  while (digitalRead(CONFIRMBUTTONPIN))
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

    //apply law of sines as well as right triangle maths to solve for temp
    airLine = ((float)bottomLine * sin((float)coalAngle * PI / 180)) / sin(((float)180 - coalAngle - airAngle) * PI / 180);
    tempLine = sin((float)airAngle * PI / 180) * airLine;
    servoMove(map((int)tempLine, 0, 1374, 255, 0));
    delay(5);
  }

  if (abs(tempLine - 926) < 50 && abs(airAngle - coalAngle) < 5)
  {
    //Serial.println("SUCCESS");
    digitalWrite(P1GLED, LOW);
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

  while (digitalRead(CONFIRMBUTTONPIN))
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

  digitalWrite(P2GLED, LOW);
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

  while (digitalRead(CONFIRMBUTTONPIN))
  {
    if (phaseChange) return 1;
    steamRead = encoderRead('C');
    if (steamRead)
    {
      steam += steamRead;
      CoalandSteam.write(0);
      if (steam > steamPrev)
      {
        digitalWrite(DIRPIN, LOW);
        for (int i = steamPrev; i < steam; i++)
        {
          stepperTick();
          delay(5);
        }
        steamPrev = steam;
      } else if (steam < steamPrev)
      {
        digitalWrite(DIRPIN, HIGH);
        for (int i = steamPrev; i > steam; i--)
        {
          stepperTick();
          delay(5);
        }
        steamPrev = steam;
      }
    }

  }
  digitalWrite(P3GLED, LOW);
  digitalWrite(P4GLED, LOW);
  digitalWrite(ENPIN, HIGH);
  digitalWrite(LIGHTBULBSWITCHPIN, HIGH);
  return 10; //temporarily skip phase four
}

byte phaseFour()
{
  /*
     This function is the fourth phase of the display.
     This phase currently does nothing.
  */
  if (!serialResponse("PHASE FOUR")) error();

  while (digitalRead(CONFIRMBUTTONPIN))
  {
    if (phaseChange) return 1;
  }
  return 10;
}

bool serialResponse(char com[])
{
  /*
     This function takes a predefined string command and confirms a serial response from a computer running processing sketch.
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
  while (!digitalRead(PHONESWITCHPIN)) {};
  if (!serialResponse("FAILURE")) error();
  //tmrpcm.disable();
  fail_state_audio();
  delay(5000);
  tmrpcm.disable();

  while (digitalRead(PHONESWITCHPIN)) {};
}

byte completion()
{
  /*
     This function is the completion state of the display.
  */
  if (!serialResponse("COMPLETE")) error();
  delay(7500);
  return 0;
}

void error()
{
  /*
     This function is the error state of the display. Only call this if a reset is necessary.
  */
  Serial.println("CRITICAL ERROR");
  while (1)
  {
    digitalWrite(LED_ON_BOARD, HIGH);
    delay(200);
    digitalWrite(LED_ON_BOARD, LOW);
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
  analogWrite(SERVOPIN, position);
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
  digitalWrite(P1RLED, !(State & 0b00000001));
  //digitalWrite(23, !(State&0b00000010));
  digitalWrite(P2RLED, !(State & 0b00000100));
  //digitalWrite(25, !(State&0b00001000));
  digitalWrite(P3RLED, !(State & 0b00010000));
  //digitalWrite(27, !(State&0b00100000));
  digitalWrite(P4RLED, !(State & 0b01000000));
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
  pinMode(STEPPIN, OUTPUT);
  pinMode(DIRPIN, OUTPUT);
  pinMode(ENPIN, OUTPUT);
  pinMode(HOMEPIN, INPUT);

  // Set motor direction clockwise
  digitalWrite(DIRPIN, HIGH);
  // disable stepper
  digitalWrite(ENPIN, HIGH);
}

void homeStepper()
{
  digitalWrite(ENPIN, LOW);

  while (!digitalRead(HOMEPIN))
  {
    stepperTick();
    delay(4);
  }
  stepperPosition = 0;
}

void stepperTick()
{
  digitalWrite(STEPPIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(STEPPIN, LOW);
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
