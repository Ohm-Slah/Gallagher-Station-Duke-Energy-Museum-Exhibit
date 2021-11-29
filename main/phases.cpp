/*
 * File name:         "phases.cpp"
 * Contributor(s):    Elliot Eickholtz, Matthew Wrocklage, Jackson Couch
 * Last edit:         11/22/21
 * Code usage:
 * This is a file containing all functions used in each of the five phases of the "main.ino" file.
 * 
 */

#include "phases.h"
// Create a new servo object:
Servo myservo;
// Create a variable to store the servo position:
int angle = 0;

Encoder AirandVoltage(2, 3);        
Encoder Coal(18, 19);       /* Creates an Encoder object, using 2 pins. Creates mulitple Encoder objects, where each uses its own 2 pins. The first pin should be capable of interrupts. 
                             * If both pins have interrupt capability, both will be used for best performance. 
                             * Encoder will also work in low performance polling mode if neither pin has interrupts. 
                             */

TM1637 tm(4, 5);            /* Library instantiation for 7-segment display
                             * Pin 4 -> DIO
                             * Pin 5 -> CLK
                             */

const uint16_t halfTwoSec = 31250;
const uint16_t fullFourSec = 62500;
byte ledState = 0;
long ledCount = 0;

TMRpcm tmrpcm; //This is for the audio 

void initialization() 
{
  /*
   * This fuction is run once on startup. 
   * This is to simply initialize everything needed.
   */

  ServoSetup();

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
  
  TCCR1A = 0;
  TIMSK1 = (1 << OCIE1A);
  sei();
  Serial.begin(9600);

  delay(100);
  
  if (!serialResponse("RESPOND")) error();
                                  //Audio 
  tmrpcm.speakerPin=9;            //5,6,11 or 46 on Mega, 9 on Uno, Nano, etc
                                //Complimentary Output or Dual Speakers:
                                //pinMode(10,OUTPUT); Pin pairs: 9,10 Mega: 5-2,6-7,11-12,46-45 
  Serial.begin(9600);
  if(!SD.begin(SD_ChipSelectPin))
  {
    Serial.println("SD fail");
    return;
  }
  
}

bool phaseZero() 
{
  /*
   * This fuction is the 'awaiting user input' phase. This will run for a maximum of three hours,
   * and will then enter a sleep state. The only difference in the sleep state is what the raspberry pi displays.
   * When exiting the sleep state, homing everthing like the initialization stage is needed.
   */
  if (!serialResponse("PHASE ZERO")) error();
  delay(1000);
  return true;

}

bool phaseOne() 
{
  /*
   * This function is the first phase of the display.
   */
  if (!serialResponse("PHASE ONE")) error();
  delay(1000);
  return true;
}

bool phaseTwo() 
{
  /*
   * This function is the second phase of the display.
   */
  if (!serialResponse("PHASE TWO")) error();
  delay(1000);
  return true;
}

bool phaseThree() 
{
  /*
   * This function is the third phase of the display.
   */
  if (!serialResponse("PHASE THREE")) error();
  delay(1000);
  return true;
}

bool phaseFour() 
{
  /*
   * This function is the fourth phase of the display.
   */
  if (!serialResponse("PHASE FOUR")) error();
  delay(1000);
  return true;
}

bool serialResponse(char com[])
{
  /*
   * This function takes a predefined string command and confirms a serial response from the raspberry pi running processing.
   * Predefined commands: "RESPOND" "PHASE ZERO" "PHASE ONE" "PHASE TWO" "PHASE THREE" "PHASE FOUR" "FAILURE" "COMPLETE"
   */
  uint8_t attempts = 0;
 
  delay(100);
  while(attempts <= 5)
  {
    Serial.println(com);
    if (Serial.available())
    {
      char val = Serial.read();
      Serial.println(val);
      if (val == '1')
      {
        return true;
      } else {
        return false;
        attempts++;
      }
    }
    delay(100);
    attempts++;
  }
  return false;
  
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

void ServoSetup() //Run these once 
{
  /*
   * This function initializes a servo, run once on startup.
   */
  myservo.attach(servoPin);
  
  for (angle = 0; angle <= 180; angle += 1) {
    myservo.write(angle);
    delay(10);
  }
  // And back from 180 to 0 degrees:
  for (angle = 180; angle >= 0; angle -= 1) 
  {
    myservo.write(angle);
    Serial.println(angle);
    delay(10);
  }
}


void ServoMove(uint16_t position)
{
  /*
   * This function recieves a position value and moves the servo to that position.
   */
  myservo.write(position);  


int8_t encoderRead(char enc) 
{
  /*
   * This function takes in a character representing what encoder value you want returned. That value is then returned.
   */
  if (enc = 'A')                  //if Air Control
  {
    return AirandVoltage.read();  //returns the accumlated position (new position)
  } else if (enc = 'C')           //if Coal Control
  {
    return Coal.read();           //returns the accumlated position (new position)
  } else if (enc = 'V')           //if Voltage Control
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
   digitalWrite(22, (State&00000001));
   digitalWrite(23, (State&00000010));
   digitalWrite(24, (State&00000100));
   digitalWrite(25, (State&00001000));
   digitalWrite(26, (State&00010000));
   digitalWrite(27, (State&00100000));
   digitalWrite(28, (State&01000000));
   digitalWrite(29, (State&10000000));
   Serial.println(State);
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
                       // run repeatedly:
tmrpcm.setVolume(6);
tmrpcm.play("JA.wav");
delay(5000); 

}
