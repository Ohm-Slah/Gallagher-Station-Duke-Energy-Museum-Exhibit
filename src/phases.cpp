/*
 * File name:         "phases.cpp"
 * Contributor(s):    Elliot Eickholtz, Matthew Wrocklage, Jackson Couch, Andrew Boehm
 * Last edit:         1/23/22
 * 
 * Code usage:
 * This is a file containing all functions used in each of the five phases of the "main.cpp" file.
 * 
 * Datasheets:
 * Rotary encoders: https://www.ctscorp.com/wp-content/uploads/288.pdf
 * 
*/

//---------------------------------------------------------------------//

#include "phases.h"
// TODO rework phases 2
// TODO write phases 3 and 4

// TODO object orient phone code for greater readability

// reinstantiate global use variables
volatile bool phaseChange = false;
volatile byte currentPhase = 0;
volatile long long lastResponse = 0;

// Create a variable to store the servo position
int angle = 0;

// Create an Encoder instances, using 2 pins each.
Encoder Air(ENCODER1APIN, ENCODER1BPIN); 
Encoder Coal(ENCODER2APIN, ENCODER2BPIN); 
Encoder Voltage(ENCODER3APIN, ENCODER3BPIN);
Encoder Govenor(ENCODER4APIN, ENCODER4BPIN);

// Library instantiation for 7-segment display
//TODO Write new 7-seg implementation code
TM1637 tm(SEGCLK, SEGDIO);  

// Create SD Card audio instance
TMRpcm tmrpcm; 

// Create LED instances that can be modified individually with minimal interaction
TimedBlink GreenLED1(P1GLED);
TimedBlink GreenLED2(P2GLED);
TimedBlink GreenLED3(P3GLED);
TimedBlink GreenLED4(P4GLED);
TimedBlink RedLED1(P1RLED);
TimedBlink RedLED2(P2RLED);
TimedBlink RedLED3(P3RLED);
TimedBlink RedLED4(P4RLED);

// variables for the stepper motor
// TODO object orient stepper motor code for increased readability
const int stepsPerRevolution = 200;
long stepperPosition;

//---------------------------------------------------------------------//

void initialization()
{
  /*
   * This fuction is run once on startup.
   * This is to simply initialize everything needed.
  */
  pinMode(PHONESWITCHPIN, INPUT_PULLUP); //Phone Switch
  pinMode(LIGHTBULBSWITCHPIN, OUTPUT); //Light bulb
  //pinMode(31, INPUT); //Send Power button
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

  lastResponse = millis();

  sei();
  Serial.begin(9600);
  delay(100);

  StepperSetup();
  initSevenSegment();
  tmrpcm.speakerPin = AUDIOPIN;

  if (!SD.begin(SDCSPIN))
  {
    Serial.println("NO SD CARD");
    error();
  }

  if (!serialResponse("RESPOND")) error();

}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void reset()
{
  /*
   * This function can be ran when all items on board need to be reset
   */
  
  homeStepper();
  phaseChangeLEDState(0);

  Air.write(1);
  Coal.write(1);

  RedLED1.blinkOff();
  RedLED2.blinkOff();
  RedLED3.blinkOff();
  RedLED4.blinkOff();
  GreenLED1.blinkOff();
  GreenLED2.blinkOff();
  GreenLED3.blinkOff();
  GreenLED4.blinkOff();

  digitalWrite(LIGHTBULBSWITCHPIN, LOW);
  //servoMove((int)10);
  //delay(100);
  //Serial.println("RIGHT HERE");
  setDCMotor(0);
  tm.clearScreen();
  
}

void deepSleep()
{
  /*
      This function is called after a great length of time has passed
      with no interaction with the controls. Ideally, this would only
      be called if the display was kept on after closing time at the 
      mueseum. This is simply an attempt to save on power consumption.
  */
  if (!serialResponse("SLEEP")) error();
  while (!phaseChange);
}

//---------------------------------------------------------------------//

byte phaseZero()
{
  /*
   *  This fuction is the 'awaiting user input' phase. This will run for a maximum of X hours,
   *  and will then enter a sleep state. The only difference in the sleep state is what the raspberry pi displays.
   *  When exiting the sleep state, reseting function is needed.
   *  This currently does nothing.
  */
  if (!serialResponse("PHASE ZERO")) error();
  while (!phaseChange) if (lastResponse + SLEEPTIME < millis()) deepSleep();
  return 1;

}

// TODO add update blink virtually everywhere you can

byte phaseOne()
{
  /*
   *  This function is the first phase of the display.
   *  Steps:
   *  play intro vid
   *  play phase 1 instruction vid
   *  balance air and coal encoders until temp servo gauge is at specified position,
   *  simultaneously waiting for confirm button to be pressed
   *  if result on servo gauge is within error margins, continue onto next phase.
   *  if result is outside error margins, play failure video
   * 
   *  Conceptual diagram:
   *  https://github.com/Ohm-Slah/Gallagher-Station-Duke-Energy-Museum-Exhibit/blob/Phase-1-Update/Pictures/phase_one_conceptual.png
   * 
   *  Conceptual Explanation:
   *  The looping portion of this function takes in 2 rotary encoder positions and limits their values to a range of 0-70 (yellow).
   *  Using Law of Sines and right angle maths, the airLine can be calculated, followed by the tempLine target value.
   *  An instability factor is added to this final value depending on an arbitrarily set "optimal value".
   *  This will sway the needle back and forth from the set point at a magnitude equal to the difference of the two user inputs.
   *  This complicated process is to allow a more dynamic output, and to increase the difficulty of an otherwise easy task.
   * 
  */

  // Begin introduction video
  if (!serialResponse("INTRO")) error();

  // Make sure everything is at an off state while the intro plays
  reset();

  // initialize variables for finding tempLine
  int8_t coalRead = 0;
  int8_t airRead = 0;
  int16_t coalAngle = 0;
  int16_t airAngle = 0;
  float airLine = 0;
  uint16_t bottomLine = 1000; //arbitrary
  float tempLine = 0;
  // optimum temp of boiler: 2150 degF

  // variables used for instability factor, or 'sway'
  bool dir = false;  //0=left & 1=right
  uint8_t count = 0;

  // Wait until intro video is finished playing, or until confirm button
  // is pressed. This action will skip the intro video.
  while(!serialWait() && digitalRead(CONFIRMBUTTONPIN)) blinkUpdate();

  // Reset lastResonse to avoid resetting to phaseZero due to inactivity
  // after watching the introduction video.
  lastResponse = millis();

  // Begin phase 1 video.
  if (!serialResponse("PHASE ONE")) error();

  // change state of external LEDs to phase 1 state.
  phaseChangeLEDState(1);

  // loop until confirm button is pressed
  while (digitalRead(CONFIRMBUTTONPIN))
  {
    blinkUpdate();  //call frequently to update blink state of all leds

    // phaseChange set in interrupt service routine @ resetPhases() 
    // ISR called when knife-switch (reset) state is changed
    if (phaseChange) return 1;  

    // if WAITTIME milliseconds have passed since the last interaction, enter phase 0
    if (lastResponse + WAITTIME < millis()) return 0;

    // read encoder positional values. Encoder position accrues automatically with interrupts
    coalRead = encoderRead('C');  //'C' = coal
    airRead = encoderRead('A');   //'A' = air

    // restricts 'sway' of output for instability factor to difference of inputs
    if (count > abs(airAngle - coalAngle) * 50) dir = !dir;

    // applies instability factor ot base line, 1000 is arbitrarily chosen
    bottomLine = 1000 - count;

    // increment/decrement instability factor according to direction of sway
    if (dir)
      count++;
    else
      count--;

    // This block limits coalRead (rotary encoder) to a range of 0-70 //
    if (coalRead)
    {
      lastResponse = millis();
      coalAngle += coalRead;
      Coal.write(0);
      if (coalAngle > 70)
        coalAngle = 70;
      else if (coalAngle < 0)
        coalAngle = 0;
    }
    //----------------------------------------------------------------//

    // This block limits airRead (rotary encoder) to a range of 0-70 //
    if (airRead)
    {
      lastResponse = millis();
      airAngle += airRead;
      Air.write(0);
      if (airAngle > 70)
        airAngle = 70;
      else if (airAngle < 0)
        airAngle = 0;
    }
    //---------------------------------------------------------------//

    // apply law of sines as well as right triangle maths to solve for tempLine //
    airLine = ((float)bottomLine * sin((float)coalAngle * PI / 180)) / sin(((float)180 - coalAngle - airAngle) * PI / 180);
    tempLine = sin((float)airAngle * PI / 180) * airLine;
    //--------------------------------------------------------------------------//

    //map tempLine to corespnding value on gauge. Servo range is 0-255
    servoMove(map((int)tempLine, 0, 1374, 255, 0));
    delay(5);
  }

  
  //Serial.println(tempLine); // uncomment for debugging

  // if tempLine was within arbitrary error margins, move on to phaseTwo().
  // otherwise, move to failure()
  if (abs(tempLine - 970) < 50 && abs(airAngle - coalAngle) < 6)
  {
    digitalWrite(P1GLED, LOW);  // TODO what is this
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
  
  Coal.write(0);
  tm.colonOff();
  int16_t steamRead = 0;
  int16_t steam = 23;
  phaseChangeLEDState(2);
  setDCMotor(23);

  while (digitalRead(CONFIRMBUTTONPIN))
  {
    if (phaseChange) return 1;
    if (lastResponse + WAITTIME < millis()) return 0;
    
    if (steamRead)
    {
      lastResponse = millis();
      steam += steamRead;
      Coal.write(0);
      if (steam > 75)
      {
        steam = 75;
      }
      else if (steam < 23)
      {
        steam = 23;
      }
      tm.clearScreen();
      setDCMotor(steam);
      tm.display(map(steam, 23, 75, 30, 66), true, false, 0);
      
    }


    steamRead = encoderRead('C');
  }
  Serial.println(steam);
  if (abs(steam - 67) < 3)
  {
    //Serial.println("SUCCESS");
    digitalWrite(P2GLED, LOW);
    setDCMotor(64);
    return 3;
  } else {
    failure();
    return 2;
  }
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
  phaseChangeLEDState(3);
  delay(1000);
  Coal.write(0);
  int16_t steamRead = 0;
  int16_t steam = 0;
  int16_t steamPrev = 0;

  while (digitalRead(CONFIRMBUTTONPIN))
  {
    if (phaseChange) return 1;
    if (lastResponse + WAITTIME < millis()) return 0;
    steamRead = encoderRead('C');
    if (steamRead)
    {
      lastResponse = millis();
      steam += steamRead;
      Coal.write(0);
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
  phaseChangeLEDState(4);
  while (digitalRead(CONFIRMBUTTONPIN))
  {
    if (phaseChange) return 1;
  }
  return 10;
}

//---------------------------------------------------------------------//

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

bool serialWait()
{
  if(Serial.available())
  {
    char val = Serial.read();
    if (val == '1') 
      return true;
    else error();
  } 
  else return false;
}

void failure()
{
  /*
     This function is the failure state of the display.
  */
  delay(1000);
  if (!serialResponse("RING")) error();
  while (!digitalRead(PHONESWITCHPIN)) {
    if (phaseChange) return;
  }
  if (!serialResponse("FAILURE")) error();
  //tmrpcm.disable();
  fail_state_audio();
  delay(5000);
  tmrpcm.disable();

  while (digitalRead(PHONESWITCHPIN)) {
    if (phaseChange) return;
  }
}

byte completion()
{
  /*
     This function is the completion state of the display.
  */
  if (!serialResponse("COMPLETE")) error();
  phaseChangeLEDState(10);
  delay(7500);
  return 0;
}

void error()
{
  /*
     This function is the error state of the display. Only call this if a reset is necessary.
  */
  Serial.println("CRITICAL ERROR");
  for(int i=0; i<20; i++)
  {
    digitalWrite(LED_ON_BOARD, HIGH);
    delay(200);
    digitalWrite(LED_ON_BOARD, LOW);
    delay(200);
  }
  resetFunc();  //call reset
}

void resetPhases()
{
  /*
     This function is attached to an interrupt, and resets any progress in the phases, bringing you back you phase 1.
  */
  phaseChange = true;
  currentPhase = 1;
}

//---------------------------------------------------------------------//

void blinkUpdate()
{
  /*
   *  This function must be called frequently to update thw state of all LED instances for blinking.
  */
  GreenLED1.blink();
  GreenLED2.blink();
  GreenLED3.blink();
  GreenLED4.blink();
  RedLED1.blink();
  RedLED2.blink();
  RedLED3.blink();
  RedLED4.blink();
}

void phaseChangeLEDState(uint8_t phase)
{
  /*
   *  This function changes the state of all of the led's according to what phase number is sent to it.
  */
  switch (phase)
  {
    case 0:
    {
      GreenLED1.blinkOff();
      GreenLED2.blinkOff();
      GreenLED3.blinkOff();
      GreenLED4.blinkOff();
      RedLED1.blinkOff();
      RedLED2.blinkOff();
      RedLED3.blinkOff();
      RedLED4.blinkOff();
    }
    case 1:
    {
      GreenLED1.blinkOff();
      GreenLED2.blinkOff();
      GreenLED3.blinkOff();
      GreenLED4.blinkOff();
      RedLED1.blink(500, 500);
      RedLED2.blinkOff();
      RedLED3.blinkOff();
      RedLED4.blinkOff();
    }
    case 2:
    {
      GreenLED1.blinkOff();
      GreenLED2.blinkOff();
      GreenLED3.blinkOff();
      GreenLED4.blinkOff();
      RedLED1.blinkOff(); digitalWrite(P1GLED, LOW);
      RedLED2.blink(500, 500);
      RedLED3.blinkOff();
      RedLED4.blinkOff();
    }
    case 3:
    {
      GreenLED1.blinkOff();
      GreenLED2.blinkOff();
      GreenLED3.blinkOff();
      GreenLED4.blinkOff();
      RedLED1.blinkOff(); digitalWrite(P1GLED, LOW);
      RedLED2.blinkOff(); digitalWrite(P1GLED, LOW);
      RedLED3.blink(500, 500);
      RedLED4.blinkOff();
    }
    case 4:
    {
      GreenLED1.blinkOff();
      GreenLED2.blinkOff();
      GreenLED3.blinkOff();
      GreenLED4.blinkOff();
      RedLED1.blinkOff(); digitalWrite(P1GLED, LOW);
      RedLED2.blinkOff(); digitalWrite(P1GLED, LOW);
      RedLED3.blinkOff(); digitalWrite(P1GLED, LOW);
      RedLED4.blink(500,500);
    }
    case 10:  //phase 'complete'
    {
      GreenLED1.blinkOff();
      GreenLED2.blinkOff();
      GreenLED3.blinkOff();
      GreenLED4.blinkOff();
      RedLED1.blinkOff(); digitalWrite(P1GLED, LOW);
      RedLED2.blinkOff(); digitalWrite(P1GLED, LOW);
      RedLED3.blinkOff(); digitalWrite(P1GLED, LOW);
      RedLED4.blinkOff(); digitalWrite(P1GLED, LOW);
    }
  }
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
  if (enc == 'A')           //if Air Control
  {
    return Air.read();      //returns the accumlated position (new position)
  } else if (enc == 'C')    //if Coal Control
  {
    return Coal.read();     //returns the accumlated position (new position)
  } else if (enc == 'V')    //if Voltage Control
  {
    return Air.read();      //returns the accumlated position (new position)
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

void fail_state_audio()
{
  tmrpcm.setVolume(6);
  tmrpcm.play("JA.wav");
  //delay(5000);
}
