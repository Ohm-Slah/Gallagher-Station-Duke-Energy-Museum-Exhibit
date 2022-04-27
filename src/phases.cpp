/*
 * File name:         "phases.cpp"
 * Contributor(s):    Elliot Eickholtz, Matthew Wrocklage, Jackson Couch, Andrew Boehm
 * Last edit:         4/21/22
 * 
 * Code usage:
 * This is a file containing all functions used in each of the five phases of the "main.cpp" file.
 * See 'phases.h' for instantiation information.
 * 
 * Datasheets:
 * Rotary encoders:   https://www.ctscorp.com/wp-content/uploads/288.pdf
 * DC Motor Driver:   https://www.pololu.com/product-info-merged/2136
 * DC Motor:          https://www.johnsonelectric.com/-/media/files/product-technology/motion/dc-motors/industry-dc-motors/low-voltage-dc-motors/20-25/pc280lg-302-imperial.ashx
 * Audio Driver PCB:  https://cdn-learn.adafruit.com/downloads/pdf/stereo-3-7w-class-d-audio-amplifier.pdf
 * SD Card Reader:    https://media.digikey.com/pdf/Data%20Sheets/DFRobot%20PDFs/DFR0229_Web.pdf
 * ATTiny4313:        https://ww1.microchip.com/downloads/en/DeviceDoc/doc8246.pdf
*/

#include "phases.h"

// Begin with some setup of instances and variables not defined in "phases.h" //
//------------------------------Start of Block--------------------------------//

// instantiate global use variables
volatile bool phaseChange = false;
volatile byte currentPhase = 0;
volatile long long lastResponse = 0;

// Create an Encoder instances, using 2 pins each.
Encoder Air(ENCODER1APIN, ENCODER1BPIN); 
Encoder Coal(ENCODER4APIN, ENCODER4BPIN); 
Encoder Reostat(ENCODER2APIN, ENCODER2BPIN);
Encoder Govenor(ENCODER3APIN, ENCODER3BPIN);

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
TimedBlink REOSTATLED(REOSTATLEDPIN);
TimedBlink STEAMLED(STEAMLEDPIN);

// Create stepper motor instance to control named 'Synchroscope'
Stepper Synchroscope;

// Create stepper motor instance to control named 'SSDisplay'
SevenSegmentDisplay SSDisplay;

// Create instance for sd card reading named 'audio'
TMRpcm audio; 

// Create instance for sd card Files named 'root'
File root;

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
  pinMode(PHONESWITCHPIN, INPUT_PULLUP);    // Pressure switch on phone, with internal PULLUP resistor
  pinMode(LIGHTBULBSWITCHPIN, OUTPUT);      // 120V lighbulb control at top of display
  pinMode(P1LEDPIN, OUTPUT);                // Red-cap Phase 1 LED
  pinMode(P2LEDPIN, OUTPUT);                // Red-cap Phase 2 LED
  pinMode(P3LEDPIN, OUTPUT);                // Red-cap Phase 3 LED
  pinMode(P4LEDPIN, OUTPUT);                // Red-cap Phase 4 LED
  pinMode(CONFIRMBUTTONLEDPIN, OUTPUT);     // Yellow-cap LED by green confirm button
  pinMode(SENDPOWERBUTTONLEDPIN, OUTPUT);   // Yellow-cap LED by red send-power button
  pinMode(MAINSWITCHLEDPIN, OUTPUT);        // Green-cap LED by knife switch
  pinMode(COALLEDPIN, OUTPUT);              // Yellow-cap LED by coal rotary encoder
  pinMode(AIRLEDPIN, OUTPUT);               // Yellow-cap LED by air rotary encoder
  pinMode(REOSTATLEDPIN, OUTPUT);           // Yellow-cap LED by rheostat rotary encoder 
  pinMode(STEAMLEDPIN, OUTPUT);             // Yellow-cap LED by governor rotary encoder
  pinMode(LED_ON_BOARD, OUTPUT);            // LED pin on Arduino Mega
  pinMode(MOTOR_PIN, OUTPUT);               // DC-motor PWM pin
  pinMode(TEMPERATURESERVOPIN, OUTPUT);     // Temperature gauge servo PWM pin
  pinMode(AMPERAGESERVOPIN, OUTPUT);        // Amperage gauge servo PWM pin
  pinMode(RESETSWITCHPIN, INPUT_PULLUP);    // Knife switch reset pin with internal PULLUP resistor
  // Attach an ISR to knife switch on FALLING edge, calling the resetPhases() function when triggered
  attachInterrupt(digitalPinToInterrupt(RESETSWITCHPIN), resetPhases, FALLING);
  pinMode(CONFIRMBUTTONPIN, INPUT_PULLUP);  // Green momentary pushbutton with internal PULLUP resistor
  pinMode(SENDPOWERBUTTONPIN, INPUT_PULLUP);// Red momentary pushbutton with internal PULLUP resistor

  pinMode(ENCODER1APIN, INPUT_PULLUP);
  pinMode(ENCODER1BPIN, INPUT_PULLUP);
  pinMode(ENCODER2APIN, INPUT_PULLUP);
  pinMode(ENCODER2BPIN, INPUT_PULLUP);
  pinMode(ENCODER3APIN, INPUT_PULLUP);
  pinMode(ENCODER3BPIN, INPUT_PULLUP);
  pinMode(ENCODER4APIN, INPUT_PULLUP);
  pinMode(ENCODER4BPIN, INPUT_PULLUP);

  // Set timeout variable to current time in milliseconds
  lastResponse = millis();

  sei();              // Enable interrupts
  Serial.begin(9600); // Begin Serial communication at 9600 BAUD rate
  delay(100);         // Delay 200 ms for serial com to begin
  initSDCard();       // Initialize SD Card and audio setup

  Synchroscope.homeStepper();

  //disablePlayback();  // Disable audio playback through phone speaker

  // Clear 7-segment display
  SSDisplay.display("      ", 0);

  // Attempt Serial communcation call-response with Raspberry Pi
  if (!serialResponse("RESPOND")) error();

}

void reset()
{
  /*
   * This function can be ran when all items need to be reset
   */
  
  Synchroscope.homeStepper();

  Air.write(1); // Reset encoder positional values
  Coal.write(1);
  Reostat.write(1);
  Synchroscope.disable();
  disableDCMotor();

  //disablePlayback();  // Disable audio playback through phone speaker

  digitalWrite(LIGHTBULBSWITCHPIN, LOW);  // turn off 120V light
  
  setDCMotor(0);                          // Set DC motor speed to 0

  SSDisplay.display("      ", 0);         // clear 7-segment display
  
}

void deepSleep()
{
  /*
   *  This function is called after a great length of time has passed
   *  with no interaction with the controls. Ideally, this would only
   *  be called if the display was kept on after closing time at the 
   *  meuseum. This is simply an attempt to save on power consumption.
  */
  if (!serialResponse("DEEP SLEEP")) error();
  while (!phaseChange) updateLEDS();
}

void intro()
{
  /*
   *  This function is ran to attempt to play the intro video.
  */
  // Do nothing until the knife switch is seated
  while(digitalRead(RESETSWITCHPIN));

  

  // Make sure everything is at an off state while the intro plays
  reset();

  // Begin introduction video
  if (!serialResponse("INTROS")) error();
  delay(100);

  phaseChange = false;  // reset 'reset' flag

  // Wait until intro video is finished playing, or until confirm button
  // is pressed. This action will skip the intro video.
  while(!serialWait() && digitalRead(CONFIRMBUTTONPIN)) {
    updateLEDS();
    if (phaseChange) return;
  } 
  delay(100);
  while(!digitalRead(CONFIRMBUTTONPIN));  // Do nothing while button is held
  delay(100);
}

byte phaseZero()
{
  /*
   *  This fuction is the 'awaiting user input' phase. This will run for a maximum of X hours,
   *  and will then enter a sleep state. The only difference in the sleep state is what the raspberry pi displays.
   *  When exiting the sleep state, reseting function is needed.
   *  This currently does nothing.
  */

  reset();

  // Begin Phase 0 video
  if (!serialResponse("PHASE ZERO")) error();

  reset();

  // Disable stepper motor to save power
  Synchroscope.disable();

  // while the knife switch is not pulled, update leds
  while (!phaseChange)
  {
    updateLEDS();
    // If a large amount of time has passed, goto deepSleep mode
    if (lastResponse + SLEEPTIME < millis()) deepSleep();
  }

  // Go to phase 1
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
   *  This process is not entirely accurate to how temperature in a furnace is balanced at Gallagher Station,
   *  it does however give a user an easily understandable concept to play with,
   *  as well as not being too far out of the realm of realism.
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

  // initialize variables for finding tempLine
  int8_t coalRead = 0;
  int8_t airRead = 0;
  int16_t coalAngle = 0;
  int16_t airAngle = 0;
  float airLine = 0;
  uint16_t bottomLine = 1000; //arbitrary value
  float tempLine = 0;
  // optimum temp of boiler: 2150 degF

  // variables used for instability factor, or 'sway'
  bool dir = false;  //0=left & 1=right
  int8_t count = 0;

  // Reset lastResonse to avoid resetting to phaseZero due to inactivity
  // after watching the introduction video.
  lastResponse = millis();
  SSDisplay.display("       ", 0);         // clear 7-segment display
  
  // Begin phase 1 video.
  if (!serialResponse("PHASE ONE INTRO")) error();
  delay(100);
  // Wait until intro video is finished playing, or until confirm button
  // is pressed. This action will skip the intro video.
  while(!serialWait() && digitalRead(CONFIRMBUTTONPIN)) {
    updateLEDS();
    if (phaseChange) return 1;
  }

  //waits for confirmation button to be pressed
  delay(100);
  while(!digitalRead(CONFIRMBUTTONPIN));
  delay(100);

  lastResponse = millis();

  //if no serial response the phase does not change
  if (!serialResponse("PHASE ONE LOOP")) error();

  phaseChange = false;

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
    // if (count > (airAngle - coalAngle) * 50) dir = !dir; // TODO polish this

    // applies instability factor ot base line, 1000 is arbitrarily chosen
    //bottomLine = 1000 - count;

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
      // Serial.println(coalAngle);
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
      // Serial.println(airAngle);
    }
    //^^^^^^^^^^^^^^^^^^^^^^^^^End of Block^^^^^^^^^^^^^^^^^^^^^^^^^^//
    // TODO polish this
    // apply law of sines as well as right triangle maths to solve for tempLine
    airLine = ((float)bottomLine * sin((float)coalAngle * PI / 180)) / sin(((float)180 - coalAngle - airAngle) * PI / 180);
    tempLine = sin((float)airAngle * PI / 180) * airLine;

    // map tempLine to coresponding value on temperature gauge. Servo range is 0-255.
    // the value 1374 was found through trial and error.
    analogWrite(TEMPERATURESERVOPIN, map((int)tempLine, 0, 1374, 255, 0));
    delay(5);
    // Serial.println(tempLine); // uncomment for debugging
    // Serial.println(map((int)tempLine, 0, 1374, 255, 0)); // uncomment for debugging
  }
  delay(100);
  
  

  // if tempLine is within arbitrary error margins, move on to phaseTwo().
  // otherwise, move to a failure state
  uint8_t tempTolerance = 75;
  uint8_t balanceTolerance = 10;
  uint16_t idealtempLine = 900;   //900 = approximately 2150 degrees on dial
  uint8_t idealServoPosistion = 95;

  if ((tempLine - idealtempLine) < tempTolerance) // checks if too high
  {
    if ((tempLine - idealtempLine) > -tempTolerance)  // checks if too low
    {
      //if (abs(airAngle - coalAngle) < balanceTolerance) // checks if balance is out
      //{
        // move gauge servo to optimal value on gauge, found through trial and error
        analogWrite(TEMPERATURESERVOPIN, idealServoPosistion);
        return 2;     // Go to Phase 2 
      //}
      //failure(1, 3);  // Play failure video, Phase 1 type 3 (unstable)
      //return 1;       // Retry Phase 1
    }
    failure(1, 2);    // Play failure video, Phase 1 type 2 (low)
    
    return 1;         // Retry Phase 1
  }
  failure(1, 1);      // Play failure video, Phase 1 type 1 (high)
  return 1;           // Retry Phase 1
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
   * The frahm tachometer is physically shaken by a dc motor with an offset weight, speed controlled by PWM.
   * 
   * This process is fairly accurate to how the Gallagher Station operators influenced the rotational
   * speed of a generator. 
   * 
   * Both outputs are technically showing the same data, but one is Rotations-Per-Minute,
   * while the other shows sinusoidal revolutions per second. The latter will be more precise. 
   * When the confirm button is pressed and the value is within an arbitrary margin of error,
   * the process moves onto phase 3. Otherwise, failure code is called and phase 2 is repeated.
  */

  // begin phase 2 video.
  if (!serialResponse("PHASE TWO INTRO")) error();

  // Wait until intro video is finished playing, or until confirm button
  // is pressed. This action will skip the intro video.
  while(!serialWait() && digitalRead(CONFIRMBUTTONPIN)) 
  {
    updateLEDS();
    if (phaseChange) return 1;
  }
  delay(500);
  
  // reset govenor encoder simulated position
  Govenor.write(0);

  // create temporary variable to store and handle data. 23 is set as lowest speed for dc motor.
  int16_t steamRead = 0;
  int16_t steam = 90;
  char cstr[7];
  setDCMotor(90);
  SSDisplay.display("30     ", 2);

  if (!serialResponse("PHASE TWO LOOP")) error();

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

      if (steam > 200)
        steam = 200;
      else if (steam < 90)
        steam = 90;

      setDCMotor(steam);
      
      itoa(map(steam, 90, 200, 30, 65), cstr, 10);
      for(int i=0; i<7; i++)
      {
        if(cstr[i]==0) 
        {
          for(;i<7;i++)
          {
            cstr[i] = ' ';
          }
          break;
        }
      }
      SSDisplay.display(cstr, 2);
      delay(15);
    }
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^End of Block^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^//

    // read encoder positional value. Encoder position accrues automatically with interrupts.
    steamRead = encoderRead('G');
  }

  delay(100);
  //Serial.println(steam);  //uncomment this line for serial debugging

  // if set value shown on frahm tach and 7-seg are within error margins, continue to phase 3,
  // otherwise run failure code and repeat phase 2
  if (steam - 188 < 3)
  {
    if ((steam - 188) > -3)
    {
      setDCMotor(188);
      return 3;
    }
    failure(2, 2);
  return 2;
  }
  failure(2, 1);
  return 2;

}

byte phaseThree()
{
  /*
   * This function is the third phase of the display.
   * Steps and Explanation:
   * Play phase 3 instructional video. 
   * Simulate changing the rotor dc voltage of a rotating generator to output a 
   * nominal ac voltage from the stator.
   * 
   * User input is a rotary encoder knob to turn,
   * output that can be seen by the user is a DC voltage meter controlled by a servo motor,
   * and a 7 segment display for AC voltage.
   * 
   * This phase is fairly simple to complete for a user, but the simulated process that is on display
   * may be a bit advanced for most. The main goal of the phase is to attempt to inform the user of 
   * how a generator electrically functions and can be controlled at a rudimentary level.
   * 
   * When the confirm button is pressed and the value is within an arbitrary margin of error,
   * the process moves onto phase 4. Otherwise, failure code is called and phase 3 is repeated.
  */

  // begin phase 3 video.
  if (!serialResponse("PHASE THREE INTRO")) error();

  // Wait until intro video is finished playing, or until confirm button
  // is pressed. This action will skip the intro video.
  while(!serialWait() && digitalRead(CONFIRMBUTTONPIN)) updateLEDS();
  delay(500);

  // reset voltage encoder simulated position
  Reostat.write(0);

  SSDisplay.display("61     ", 2);

  // initialize temporary variables
  int16_t voltageRead = 0;
  int16_t voltage = 0;

  if (!serialResponse("PHASE THREE LOOP")) error();

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
      voltage += voltageRead*2;
      Reostat.write(0);

      if (voltage > 255)
        voltage = 255;
      else if (voltage < 0)
        voltage = 0;

      // ! This line must be tested to find appropriate values to display on 7-segment
      // Serial.println(voltage);

      analogWrite(AMPERAGESERVOPIN, map((int)voltage, 0, 255, 255, 0));
    }
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^End of Block^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^//

  }

  if (abs(voltage - 146) < 5)  
  {
    if ((voltage - 146) > -5)
    {
      return 4; // begin phase 4
    }
    failure(3, 2);
    return 3; // begin phase 3, again
  }
  failure(3, 1);
  return 3; // begin phase 3, again
}

byte phaseFour()
{
  /*
   * This function is the fourth phase of the display.
   * 
   * Steps and Explanation:
   * Play phase 4 instructional video. 
   * Simulate a process of connecting the generator voltage output to the electrical grid.
   * User input is a 'send power' push button,
   * output to the user is a synchroscope controlled be a stepper motor.
   * 
   * The synchroscope slowly rotates clockwise until the 'send power' button is pressed.
   * This phase is about timing the button press when the syncroscope points at a specific location.
   * 
   * When the 'send power' button is pressed and the value is within an arbitrary margin of error,
   * the process is finished and calls completion(). Otherwise, failure code is called and phase 4 is repeated.
  */

  // begin phase 4 video.
  if (!serialResponse("PHASE FOUR INTRO")) error();

  // Wait until intro video is finished playing, or until confirm button
  // is pressed. This action will skip the intro video.
  while(!serialWait() && digitalRead(CONFIRMBUTTONPIN)) updateLEDS();
  delay(500);

  if (!serialResponse("PHASE FOUR LOOP")) error();

  Synchroscope.homeStepper();

  lastResponse = millis();

  Serial.println("Stepper Homed");
  delay(250);
  
  phaseChange = false;

  SSDisplay.display("61     ", 2);

  // loop until confirm button is pressed
  while (digitalRead(SENDPOWERBUTTONPIN))
  {
    //Serial.print("PHASE 4 LOOP");
    // phaseChange set in interrupt service routine @ resetPhases() 
    // ISR called when knife-switch (reset) state is changed
    if (phaseChange) return 1;

    // if WAITTIME milliseconds have passed since the last interaction, enter phase 0
    if (lastResponse + WAITTIME < millis()) return 0;

    // update led blinking states
    updateLEDS();

    Synchroscope.singleStep(false);
    delay(7); // this delay will need to be adjusted to change difficulty
  }

  lastResponse = millis();

  //! 
  return 10;

  // ! This code below must be tested to find actual error margins //
  // !-------------------------Start of Block----------------------//
  Serial.println(Synchroscope.stepperPosition);
  if((Synchroscope.stepperPosition) > 5)
  {
     if ((Synchroscope.stepperPosition-200) > -5)
     {
       return 10;
     }
     failure(4, 2);
    return 4;
   }
   failure(4, 1);
   return 4;
}
  // !^^^^^^^^^^^^^^^^^^^^^^^^^End of Block^^^^^^^^^^^^^^^^^^^^^^^^//

bool serialResponse(char com[])
{
  /*
   * This function takes a predefined string command and confirms a serial 
   * response from a Raspberry Pi running a python sketch.
   * Predefined commands: 
   * "RESPOND" "INTROS" "PHASE ZERO" "DEEP SLEEP"
   * "PHASE ONE INTRO" "PHASE ONE LOOP" "PHASE ONE FAIL HIGH" "PHASE ONE FAIL LOW"
   * "PHASE TWO INTRO" "PHASE TWO LOOP" "PHASE TWO FAIL HIGH" "PHASE TWO FAIL LOW"
   * "PHASE THREE INTRO" "PHASE THREE LOOP" "PHASE THREE FAIL HIGH" "PHASE THREE FAIL LOW"
   * "PHASE FOUR INTRO" "PHASE FOUR LOOP" "PHASE FOUR FAIL HIGH" "PHASE FOUR FAIL LOW"
   * "COMPLETE" "SLEEP"
  */
  uint8_t attempts = 0;

  delay(100);
  while (attempts <= 5)
  {
    updateLEDS();
    delay(250);
    if (Serial.available())
    {
      char val = Serial.read();
      if (val == '1' && com == "RESPOND")
      {
        return true;
      } else if (val == '2' && com == "INTROS") 
      {
        return true;
      } else if (val == '3' && com == "PHASE ZERO") 
      {
        return true;
      } else if (val == '4' && com == "DEEP SLEEP") 
      {
        return true;
      } else if (val == '5' && com == "PHASE ONE INTRO") 
      {
        return true;
      } else if (val == '6' && com == "PHASE ONE LOOP") 
      {
        return true;
      } else if (val == '7' && com == "PHASE ONE FAIL HIGH") 
      {
        return true;
      } else if (val == '8' && com == "PHASE ONE FAIL LOW") 
      {
        return true;
      } else if (val == '9' && com == "PHASE ONE UNBALANCED") 
      {
        return true;
      } else if (val == 'A' && com == "PHASE TWO INTRO") 
      {
        return true;
      } else if (val == 'B' && com == "PHASE TWO LOOP") 
      {
        return true;
      } else if (val == 'C' && com == "PHASE TWO FAIL HIGH") 
      {
        return true;
      } else if (val == 'D' && com == "PHASE TWO FAIL LOW") 
      {
        return true;
      } else if (val == 'E' && com == "PHASE THREE INTRO") 
      {
        return true;
      } else if (val == 'F' && com == "PHASE THREE LOOP") 
      {
        return true;
      } else if (val == 'G' && com == "PHASE THREE FAIL HIGH") 
      {
        return true;
      } else if (val == 'H' && com == "PHASE THREE FAIL LOW") 
      {
        return true;
      } else if (val == 'I' && com == "PHASE FOUR INTRO") 
      {
        return true;
      } else if (val == 'J' && com == "PHASE FOUR LOOP") 
      {
        return true;  
      } else if (val == 'K' && com == "PHASE FOUR FAIL HIGH") 
      {
        return true;
      } else if (val == 'L' && com == "PHASE FOUR FAIL LOW") 
      {
        return true;
      } else if (val == 'M' && com == "COMPLETE") 
      {
        return true;
      } else if (val == 'N' && com == "RING") 
      {
        return true;
      } else {
        attempts++;
      }
    }
    Serial.println(com);
    delay(250);
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
    if (val == '0') 
      return true;
    else
      return false;
  }
  return false;
}

void failure(uint8_t phase, uint8_t failureReason)
{
  /*
   * This function is the failure state of the display.
   * It takes in the phase that the failure occured at, as well as
   * the reason in which it failed. The failure states that exist are shown below:
   * 
   * 1 : Too high failure
   * 2 : Too low failure
   * 3 : Other failure
  */
  delay(100);
  if (!serialResponse("RING")) error();

  while (!digitalRead(PHONESWITCHPIN))
  {
    updateLEDS();
    if (phaseChange) return;
  }

  switch (phase) 
  {
    case 1:
      switch (failureReason)
      {
        case 1:
          if (!serialResponse("PHASE ONE FAIL HIGH")) error();
        break;
          
        case 2:
          if (!serialResponse("PHASE ONE FAIL LOW")) error();
        break;

        case 3:
          if (!serialResponse("PHASE ONE UNBALANCED")) error();
        break;
      }
    break;

    case 2:
      switch (failureReason)
      {
        case 1:
          if (!serialResponse("PHASE TWO FAIL HIGH")) error();
        break;
          
        case 2:
          if (!serialResponse("PHASE TWO FAIL LOW")) error();
        break;
      }
    break;

    case 3:
      switch (failureReason)
      {
        case 1:
          if (!serialResponse("PHASE THREE FAIL HIGH")) error();
        break;
          
        case 2:
          if (!serialResponse("PHASE THREE FAIL LOW")) error();
        break;
      }
    break;

    case 4:
      switch (failureReason)
      {
        case 1:
          if (!serialResponse("PHASE FOUR FAIL HIGH")) error();
        break;
          
        case 2:
          if (!serialResponse("PHASE FOUR FAIL LOW")) error();
        break;
      }
    break;
  }

  //playFailureAudio();
  //while(audio.isPlaying());
  //disablePlayback();

  while (digitalRead(PHONESWITCHPIN))
  {
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
  delay(500);
  digitalWrite(LIGHTBULBSWITCHPIN, HIGH);
  phaseChangeLEDState(10);
  SSDisplay.display("60     ", 2);
  analogWrite(AMPERAGESERVOPIN, 30);

  while(!serialWait() && digitalRead(CONFIRMBUTTONPIN)) {
    updateLEDS();
    if (phaseChange) return 1;
  }
  digitalWrite(LIGHTBULBSWITCHPIN, LOW);
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
  cli();
  //Serial.println("EVENT");
  phaseChange = true;
  currentPhase = 1;
  reset();
  sei();
}

void initSDCard()
{
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("failed!");
    while(true);  // stay here.
  }
  Serial.println("OK!");

  audio.speakerPin = 46;  // set speaker output to pin 46

  root = SD.open("/");      // open SD card main root
  //printDirectory(root, 0);  // print all files names and sizes


  audio.setVolume(5);    //   0 to 7. Set volume level

  audio.quality(1);      //  Set 1 for 2x oversampling Set 0 for normal

}

/*
 * Current status of phone code is the phone freezes servo motors from functioning when failure appears.
 * troubleshoot and finish later
 */

// void playFailureAudio()
// {
//   while ( !audio.isPlaying() ) {
//     // no audio file is playing
//     File entry =  root.openNextFile();  // open next file
//     if (! entry) {
//       // no more files
//       root.rewindDirectory();  // go to start of the folder
//       //return;
//     }

//     uint8_t nameSize = String(entry.name()).length();  // get file name size
//     String str1 = String(entry.name()).substring( nameSize - 4 );  // save the last 4 characters (file extension)

//     if ( str1.equalsIgnoreCase(".wav") ) {
//       // the opened file has '.wav' extension
//       audio.play( entry.name() );      // play the audio file
//       Serial.print("Playing file: ");
//       Serial.println( entry.name() );
//     }

//     else {
//       // not '.wav' format file
//       entry.close();
//       //return;
//     }
//   }
// }

// // void printDirectory(File dir, int numTabs) {
// //   while (true) {

// //     File entry =  dir.openNextFile();
// //     if (! entry) {
// //       // no more files
// //       break;
// //     }
// //     for (uint8_t i = 0; i < numTabs; i++) {
// //       Serial.print('\t');
// //     }
// //     Serial.print(entry.name());
// //     if (entry.isDirectory()) {
// //       Serial.println("/");
// //       printDirectory(entry, numTabs + 1);
// //     } else {
// //       // files have sizes, directories do not
// //       Serial.print("\t\t");
// //       Serial.println(entry.size(), DEC);
// //     }
// //     entry.close();
// //   }
// // }

// void disablePlayback()
// {
//   audio.disable();
//   //digitalWrite(AUDIOPIN, LOW);
// }

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
  REOSTATLED.blink();
  STEAMLED.blink();
  //Serial.println("LED STATE UPDATE");
}

void phaseChangeLEDState(uint8_t phase)
{
  /*
   *  This function changes the blinking state of all of the led's according to what phase number is sent to it.
  */
  switch (phase)
  {
    case 0:
      RedLED1.blinkOff();
      RedLED2.blinkOff();
      RedLED3.blinkOff();
      RedLED4.blinkOff();
      CONFIRMBUTTONLED.blinkOff();
      SENDPOWERBUTTONLED.blinkOff();
      MAINSWITCHLED.blink(500, 500);  // 500ms on / 500ms off
      COALLED.blinkOff();
      AIRLED.blinkOff();
      REOSTATLED.blinkOff();
      STEAMLED.blinkOff();
      //Serial.println("PHASE 0 LIGHTS");
    break;
    case 1:
      RedLED1.blink(800, 200);
      RedLED2.blinkOff();
      RedLED3.blinkOff();
      RedLED4.blinkOff();
      CONFIRMBUTTONLED.blink(10000, 1);
      SENDPOWERBUTTONLED.blinkOff();
      MAINSWITCHLED.blink(500, 500);
      COALLED.blink(10000, 1);
      AIRLED.blink(10000, 1);
      REOSTATLED.blinkOff();
      STEAMLED.blinkOff();
      //Serial.println("PHASE 1 LIGHTS");
      break;
    case 2:
      RedLED1.blinkOff(); 
      RedLED2.blink(800, 200);
      RedLED3.blinkOff();
      RedLED4.blinkOff();
      CONFIRMBUTTONLED.blink(10000, 1);
      SENDPOWERBUTTONLED.blinkOff();
      MAINSWITCHLED.blink(500, 500);
      COALLED.blinkOff();
      AIRLED.blinkOff();
      REOSTATLED.blinkOff();
      STEAMLED.blink(10000, 1);
    break;
    case 3:
      RedLED1.blinkOff(); 
      RedLED2.blinkOff(); 
      RedLED3.blink(800, 200);
      RedLED4.blinkOff();
      CONFIRMBUTTONLED.blink(10000, 1);
      SENDPOWERBUTTONLED.blinkOff();
      MAINSWITCHLED.blink(500, 500);
      COALLED.blinkOff();
      AIRLED.blinkOff();
      REOSTATLED.blink(10000, 1);
      STEAMLED.blinkOff();
    break;
    case 4:
      RedLED1.blinkOff(); 
      RedLED2.blinkOff();
      RedLED3.blinkOff(); 
      RedLED4.blink(800,200);
      CONFIRMBUTTONLED.blinkOff();
      SENDPOWERBUTTONLED.blink(10000, 1);
      MAINSWITCHLED.blink(500, 500);
      COALLED.blinkOff();
      AIRLED.blinkOff();
      REOSTATLED.blinkOff();
      STEAMLED.blinkOff();
    break;
    case 10:  //phase 'complete'
      RedLED1.blink();
      RedLED2.blinkOff();
      RedLED3.blinkOff(); 
      RedLED4.blinkOff();
      CONFIRMBUTTONLED.blinkOff();
      SENDPOWERBUTTONLED.blinkOff();
      MAINSWITCHLED.blinkOff();
      COALLED.blinkOff();
      AIRLED.blinkOff();
      REOSTATLED.blinkOff();
      STEAMLED.blinkOff();
    break;
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
  } else if (enc == 'V')    //if Reostat Control
  {
    return Reostat.read();   //returns the accumlated position (new position)
  }  else if (enc == 'G')    //if Govenor(steam) Control
  {
    return Govenor.read();   //returns the accumlated position (new position)
  }
}

void setDCMotor(uint16_t pwmValue)
{
  /*
   * This fuction recieves an integer value and runs the DC motor at that PWM at 255 precision.
  */
  digitalWrite(DCMOTORENPIN, HIGH);
  analogWrite(MOTOR_PIN, pwmValue);
}

void disableDCMotor()
{
  digitalWrite(DCMOTORENPIN, LOW);
}