/*
 * File name:         "phases.cpp"
 * Contributor(s):    Elliot Eickholtz, Matthew Wrocklage, Jackson Couch, Andrew Boehm
 * Last edit:         1/29/22
 * 
 * Code usage:
 * This is a file containing all functions used in each of the five phases of the "main.cpp" file.
 * 
 * Datasheets:
 * Rotary encoders: https://www.ctscorp.com/wp-content/uploads/288.pdf
 * 
*/

#include "phases.h"

// TODO object orient phone code for greater readability

// Begin with some setup of instances and variables not defined in "phases.h" //
//------------------------------Start of Block--------------------------------//

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
TM1637 tm(SEGCLKPIN, SEGDIOPIN);  

// Create SD Card audio instance
TMRpcm tmrpcm; 

// Create LED blinking instances tied to their pinout that can be modified individually with minimal interaction
TimedBlink RedLED1(P1LEDPIN);
TimedBlink RedLED2(P2LEDPIN);
TimedBlink RedLED3(P3LEDPIN);
TimedBlink RedLED4(P4LEDPIN);
TimedBlink CONFIRMBUTTONLED(CONFIRMBUTTONLEDPIN);
TimedBlink SENDPOWERBUTTONLED(SENDPOWERBUTTONLEDPIN);
TimedBlink MAINSWITCHLED(MAINSWITCHLEDPIN);
TimedBlink COALLED(COALLEDPIN);
TimedBlink AIRLED(AIRLEDPIN);
TimedBlink VOLTAGELED(VOLTAGELEDPIN);
TimedBlink STEAMLED(STEAMLEDPIN);

// Create Stepper motor instance to control
Stepper Synchroscope;

// declare reset function @ address 0
// basically, calling resetFunc() is equivalent to hitting the reset button on the Arduino
void(* resetFunc) (void) = 0; 

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^End of Block^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^//

void initialization()
{
  /*
   * This fuction is run once on startup.
   * This is to simply initialize everything needed.
  */
  pinMode(PHONESWITCHPIN, INPUT_PULLUP);
  pinMode(LIGHTBULBSWITCHPIN, OUTPUT);
  pinMode(P1LEDPIN, OUTPUT); //Phase 1 LED
  pinMode(P2LEDPIN, OUTPUT); //Phase 2 LED
  pinMode(P3LEDPIN, OUTPUT); //Phase 3 LED
  pinMode(P4LEDPIN, OUTPUT); //Phase 4 LED
  pinMode(CONFIRMBUTTONLEDPIN, OUTPUT);
  pinMode(SENDPOWERBUTTONLEDPIN, OUTPUT);
  pinMode(MAINSWITCHLEDPIN, OUTPUT);
  pinMode(COALLEDPIN, OUTPUT);
  pinMode(AIRLEDPIN, OUTPUT);
  pinMode(VOLTAGELEDPIN, OUTPUT);
  pinMode(STEAMLEDPIN, OUTPUT);
  pinMode(LED_ON_BOARD, OUTPUT); //LED pin on Arduino Mega
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(TEMPERATURESERVOPIN, OUTPUT);
  pinMode(VOLTAGESERVOPIN, OUTPUT);
  pinMode(RESETSWITCHPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RESETSWITCHPIN), resetPhases, FALLING);
  pinMode(CONFIRMBUTTONPIN, INPUT_PULLUP);

  lastResponse = millis();

  sei();
  Serial.begin(9600);
  delay(100);

  Synchroscope.homeStepper();
  initSevenSegment();
  tmrpcm.speakerPin = AUDIOPIN;

  if (!SD.begin(SDCSPIN))
  {
    Serial.println("NO SD CARD");
    error();
  }

  if (!serialResponse("RESPOND")) error();

}

void reset()
{
  /*
   * This function can be ran when all items on board need to be reset
   */
  
  Synchroscope.homeStepper();
  phaseChangeLEDState(0);

  Air.write(1);
  Coal.write(1);

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
   *  This function is called after a great length of time has passed
   *  with no interaction with the controls. Ideally, this would only
   *  be called if the display was kept on after closing time at the 
   *  meuseum. This is simply an attempt to save on power consumption.
  */
  if (!serialResponse("SLEEP")) error();
  while (!phaseChange) updateLEDS();
}

byte phaseZero()
{
  /*
   *  This fuction is the 'awaiting user input' phase. This will run for a maximum of X hours,
   *  and will then enter a sleep state. The only difference in the sleep state is what the raspberry pi displays.
   *  When exiting the sleep state, reseting function is needed.
   *  This currently does nothing.
  */
  if (!serialResponse("PHASE ZERO")) error();
  while (!phaseChange) 
  {
    updateLEDS();
    if (lastResponse + SLEEPTIME < millis()) 
    {
      deepSleep();
    }
  }
  return 1;
}

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
   *  https://github.com/Ohm-Slah/Gallagher-Station-Duke-Energy-Museum-Exhibit/blob/main/Pictures/phase_one_conceptual.png
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
  while(!serialWait() && digitalRead(CONFIRMBUTTONPIN)) updateLEDS();

  // Reset lastResonse to avoid resetting to phaseZero due to inactivity
  // after watching the introduction video.
  lastResponse = millis();
  
  // Begin phase 1 video.
  if (!serialResponse("PHASE ONE")) error();

  // loop until confirm button is pressed
  while (digitalRead(CONFIRMBUTTONPIN))
  {
    // phaseChange set in interrupt service routine @ resetPhases() 
    // ISR called when knife-switch (reset) state is changed
    if (phaseChange) return 1;  

    // if WAITTIME milliseconds have passed since the last interaction, enter phase 0
    if (lastResponse + WAITTIME < millis()) return 0;

    // update phase 1 led blinking states
    updateLEDS();

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
    //------------------------Start of Block--------------------------//
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
    //^^^^^^^^^^^^^^^^^^^^^^^^^End of Block^^^^^^^^^^^^^^^^^^^^^^^^^^^//


    // This block limits airRead (rotary encoder) to a range of 0-70 //
    //------------------------Start of Block-------------------------//
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
    //^^^^^^^^^^^^^^^^^^^^^^^^^End of Block^^^^^^^^^^^^^^^^^^^^^^^^^^//

    // apply law of sines as well as right triangle maths to solve for tempLine
    airLine = ((float)bottomLine * sin((float)coalAngle * PI / 180)) / sin(((float)180 - coalAngle - airAngle) * PI / 180);
    tempLine = sin((float)airAngle * PI / 180) * airLine;

    // map tempLine to coresponding value on temperature gauge. Servo range is 0-255.
    // the value 1374 was found through trial and error.
    analogWrite(TEMPERATURESERVOPIN, map((int)tempLine, 0, 1374, 255, 0));
    delay(5);
  }

  
  //Serial.println(tempLine); // uncomment for debugging

  // if tempLine is within arbitrary error margins, move on to phaseTwo().
  // otherwise, move to failure()
  if (abs(tempLine - 970) < 50 && abs(airAngle - coalAngle) < 6)
  {
    // move gauge servo to optimal value, found through trial and error
    analogWrite(TEMPERATURESERVOPIN, 79);
    return 2;
  } else {
    failure();
    return 1;
  }

}

byte phaseTwo()
{
  /*
   * This function is the second phase of the display.
   * Steps and Explanation:
   * Play phase 2 instruction video.
   * Simulate setting the rotational speed of the rotor in the generator to a set speed.
   * Input from the user wil be a rotating govenor valve, output will be RPM on frahm tachometer
   * and simulated electrical frequency of stator shown on 7-segment display.
   * The frahm tachometer is shaken by a dc motor with an offset weight, speed controlled by PWM.
   * 
   * Both outputs are technically showing the same data, but one is Rotations-Per-Minute,
   * while the other shows sinosodial revolutions per second. The latter will be more precise. 
   * When the confirm button is pressed and the value is within an arbitrary margin of error,
   * the process moves onto phase 3. Otherwise, failure code is called and phase 2 is repeated.
  */

  // begin phase 2 video.
  if (!serialResponse("PHASE TWO")) error();
  
  // reset govenor encoder simulated position
  Govenor.write(0);
  tm.colonOff();

  // create temporary variable to store and handle data. 23 is set as lowest speed for dc motor.
  int16_t steamRead = 0;
  int16_t steam = 23;
  setDCMotor(23);

  // loop until confirm button is pressed
  while (digitalRead(CONFIRMBUTTONPIN))
  {
    // phaseChange set in interrupt service routine @ resetPhases() 
    // ISR called when knife-switch (reset) state is changed
    if (phaseChange) return 1;

    // if WAITTIME milliseconds have passed since the last interaction, enter phase 0
    if (lastResponse + WAITTIME < millis()) return 0;

    // update led blinking states
    updateLEDS();
    
    // This block limits steamRead (rotary encoder) and dc motor PWM to a range of 23-75 //
    // This is the range of values that can be shown on frahm tachometer                 //
    //----------------------------------Start of Block-----------------------------------//
    if (steamRead)
    {
      lastResponse = millis();
      steam += steamRead;
      Govenor.write(0);

      if (steam > 75)
        steam = 75;
      else if (steam < 23)
        steam = 23;

      tm.clearScreen();
      setDCMotor(steam);
      // TODO 7-segment code
      
    }
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^End of Block^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^//

    // read encoder positional value. Encoder position accrues automatically with interrupts.
    steamRead = encoderRead('G');
  }

  //Serial.println(steam);  //uncomment this line for serial debugging

  // if set value shown on frahm tach and 7-seg are within error margins, continue to phase 3,
  // otherwise run failure code and repeat phase 2
  if (abs(steam - 67) < 3)
  {
    setDCMotor(64);
    return 3;
  } else {
    failure();
    return 2;
  }
}

byte phaseThree()
{
  /*
   * This function is the third phase of the display.
   * 
   * 
  */

  // begin phase 3 video.
  if (!serialResponse("PHASE THREE")) error();

  // reset voltage encoder simulated position
  Voltage.write(0);

  // initialize temporary variables
  int16_t voltageRead = 0;
  int16_t voltage = 0;

  // loop until confirm button is pressed
  while (digitalRead(CONFIRMBUTTONPIN))
  {
    // phaseChange set in interrupt service routine @ resetPhases() 
    // ISR called when knife-switch (reset) state is changed
    if (phaseChange) return 1;

    // if WAITTIME milliseconds have passed since the last interaction, enter phase 0
    if (lastResponse + WAITTIME < millis()) return 0;

    // read encoder positional value. Encoder position accrues automatically with interrupts.
    voltageRead = encoderRead('V');

    // update led blinking states
    updateLEDS();

    // This block limits voltageRead (rotary encoder) and dc motor PWM to a range of 0-255 //
    // This is the range of values that can be shown on frahm tachometer                   //
    //----------------------------------Start of Block-------------------------------------//
    if (voltageRead)
    {
      lastResponse = millis();
      voltage += voltageRead;
      Voltage.write(0);

      if (voltage > 255)
        voltage = 255;
      else if (voltage < 0)
        voltage = 0;

      // TODO 7 segment code

      // ! This line currently inverts the value of voltage and
      // ! and moves the servo to that position, range of 0-255.
      // ! The actual postions must be found through trial and error.
      analogWrite(VOLTAGESERVOPIN, map((int)voltage, 0, 255, 255, 0));
    }
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^End of Block^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^//

  }

  // ! This code below must be tested to find actual error margins //
  // !-------------------------Start of Block----------------------//
  if (abs(voltage - 0) < 1)  
  {
    return 4; // begin phase 4
  } else 
  {
    failure();
    return 3; // begin phase 3, again
  }
  // !^^^^^^^^^^^^^^^^^^^^^^^^^End of Block^^^^^^^^^^^^^^^^^^^^^^^^//
}

byte phaseFour()
{
  /*
   * This function is the fourth phase of the display.
   * 
  */

  // begin phase 4 video.
  if (!serialResponse("PHASE FOUR")) error();

  Synchroscope.homeStepper();
  
  // loop until confirm button is pressed
  while (digitalRead(CONFIRMBUTTONPIN))
  {
    // phaseChange set in interrupt service routine @ resetPhases() 
    // ISR called when knife-switch (reset) state is changed
    if (phaseChange) return 1;

    // if WAITTIME milliseconds have passed since the last interaction, enter phase 0
    if (lastResponse + WAITTIME < millis()) return 0;

    // update led blinking states
    updateLEDS();

    Synchroscope.singleStep(true);
    delay(25); // ! this delay will need to be adjusted to change difficulty
  }

  // ! This code below must be tested to find actual error margins //
  // !-------------------------Start of Block----------------------//
  if(Synchroscope.stepperPosition > 1)
  {
    return 10;
  } else
  {
    failure();
    return 4;
  }
  // !^^^^^^^^^^^^^^^^^^^^^^^^^End of Block^^^^^^^^^^^^^^^^^^^^^^^^//
}

bool serialResponse(char com[])
{
  /*
   * This function takes a predefined string command and confirms a serial response from a computer running processing sketch.
   * Predefined commands: "RESPOND" "RING" "PHASE ZERO" "PHASE ONE" "PHASE TWO" "PHASE THREE" "PHASE FOUR" "FAILURE" "COMPLETE" "SLEEP"
  */
  uint8_t attempts = 0;

  delay(1000);
  while (attempts <= 5)
  {
    updateLEDS();
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
  /*
   * This function will return true if there was anything waiting in the USB serial buffer
  */

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
   * This function is the failure state of the display.
  */
  delay(1000);
  if (!serialResponse("RING")) error();
  while (!digitalRead(PHONESWITCHPIN)) {
    updateLEDS();
    if (phaseChange) return;
  }
  if (!serialResponse("FAILURE")) error();
  //tmrpcm.disable();
  fail_state_audio();
  delay(5000);
  tmrpcm.disable();

  while (digitalRead(PHONESWITCHPIN)) {
    updateLEDS();
    if (phaseChange) return;
  }
}

byte completion()
{
  /*
   * This function is the completion state of the display.
  */
  if (!serialResponse("COMPLETE")) error();
  phaseChangeLEDState(10);
  delay(7500);
  return 0;
}

void error()
{
  /*
   * This function is the error state of the display. Only call this if a reset is necessary.
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
   * This function is attached to an interrupt, and resets any progress in the phases, bringing you back you phase 1.
  */
  phaseChange = true;
  currentPhase = 1;
}

void updateLEDS()
{
  /*
   *  This function must be called frequently to update the slow blinking state of all LED's.
  */
  RedLED1.blink();
  RedLED2.blink();
  RedLED3.blink();
  RedLED4.blink();
  CONFIRMBUTTONLED.blink();
  SENDPOWERBUTTONLED.blink();
  MAINSWITCHLED.blink();
  COALLED.blink();
  AIRLED.blink();
  VOLTAGELED.blink();
  STEAMLED.blink();
}

void phaseChangeLEDState(uint8_t phase)
{
  /*
   *  This function changes the blinking state of all of the led's according to what phase number is sent to it.
  */
  switch (phase)
  {
    case 0:
    {
      RedLED1.blinkOff();
      RedLED2.blinkOff();
      RedLED3.blinkOff();
      RedLED4.blinkOff();
      CONFIRMBUTTONLED.blinkOff();
      SENDPOWERBUTTONLED.blinkOff();
      MAINSWITCHLED.blink(500, 500);
      COALLED.blinkOff();
      AIRLED.blinkOff();
      VOLTAGELED.blinkOff();
      STEAMLED.blinkOff();
    }
    case 1:
    {
      RedLED1.blink(500, 500);
      RedLED2.blinkOff();
      RedLED3.blinkOff();
      RedLED4.blinkOff();
      CONFIRMBUTTONLED.blink(250, 250);
      SENDPOWERBUTTONLED.blinkOff();
      MAINSWITCHLED.blink(500, 500);
      COALLED.blink(250, 250);
      AIRLED.blink(250, 250);
      VOLTAGELED.blinkOff();
      STEAMLED.blinkOff();
    }
    case 2:
    {
      RedLED1.blinkOff(); 
      RedLED2.blink(500, 500);
      RedLED3.blinkOff();
      RedLED4.blinkOff();
      CONFIRMBUTTONLED.blink(250, 250);
      SENDPOWERBUTTONLED.blinkOff();
      MAINSWITCHLED.blink(500, 500);
      COALLED.blinkOff();
      AIRLED.blinkOff();
      VOLTAGELED.blinkOff();
      STEAMLED.blink(250, 250);
    }
    case 3:
    {
      RedLED1.blinkOff(); 
      RedLED2.blinkOff(); 
      RedLED3.blink(500, 500);
      RedLED4.blinkOff();
      CONFIRMBUTTONLED.blink(250, 250);
      SENDPOWERBUTTONLED.blinkOff();
      MAINSWITCHLED.blink(500, 500);
      COALLED.blinkOff();
      AIRLED.blinkOff();
      VOLTAGELED.blink(250, 250);
      STEAMLED.blinkOff();
    }
    case 4:
    {
      RedLED1.blinkOff(); 
      RedLED2.blinkOff();
      RedLED3.blinkOff(); 
      RedLED4.blink(500,500);
      CONFIRMBUTTONLED.blinkOff();
      SENDPOWERBUTTONLED.blink(150, 150);
      MAINSWITCHLED.blink(500, 500);
      COALLED.blinkOff();
      AIRLED.blinkOff();
      VOLTAGELED.blinkOff();
      STEAMLED.blinkOff();
    }
    case 10:  //phase 'complete'
    {
      RedLED1.blinkOff();
      RedLED2.blinkOff();
      RedLED3.blinkOff(); 
      RedLED4.blinkOff();
      CONFIRMBUTTONLED.blinkOff();
      SENDPOWERBUTTONLED.blinkOff();
      MAINSWITCHLED.blinkOff();
      COALLED.blinkOff();
      AIRLED.blinkOff();
      VOLTAGELED.blinkOff();
      STEAMLED.blinkOff();
    }
  }
}

int8_t encoderRead(char enc)
{
  /*
   * This function takes in a character representing what encoder value you want returned. That value is then returned.
  */
  if (enc == 'A')           //if Air Control
  {
    return Air.read();      //returns the accumlated position (new position)
  } else if (enc == 'C')    //if Coal Control
  {
    return Coal.read();     //returns the accumlated position (new position)
  } else if (enc == 'V')    //if Voltage Control
  {
    return Voltage.read();   //returns the accumlated position (new position)
  }  else if (enc == 'G')    //if Govenor(steam) Control
  {
    return Govenor.read();   //returns the accumlated position (new position)
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

void fail_state_audio()
{
  tmrpcm.setVolume(6);
  tmrpcm.play("JA.wav");
  //delay(5000);
}
