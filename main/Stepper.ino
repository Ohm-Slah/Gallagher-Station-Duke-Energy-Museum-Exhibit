#include <Stepper.h>


// Define pin connections & motor's steps per revolution
const int dirPin = 2;
const int stepPin = 3;
const int stepsPerRevolution = 200;

void setup()
{
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}
void loop()
{
  // Set motor direction clockwise
  digitalWrite(dirPin, HIGH);

  // Spin motor slowly
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000);
  }
  delay(1000); // Wait a second
  
  // Set motor direction counterclockwise
  digitalWrite(dirPin, LOW);

  // Spin motor quickly
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
  delay(1000); // Wait a second
}




//--NEW CODE--
int totalMotorSteps = 200; 
int currentMotorStep = 0; 
int home_switch = 4; 
const int dirPin = 2;
const int stepPin = 3;

//boolean motorRun = false; 
int position ; 
void Move_home1();
void find_home(); 

void setup() 
{
  
   Rotate at a set rate until homing switch is activated and sets global varaible to zero (home position)
      while (home_switch == LOW) //When the switch is not on 
      {
       find_home();  //calls the function home
      }
      
      position = 0; //sets back to zero 
}

void loop()
{
  find_home(); 
  Move_home1(); 
}
void find_home ()
{
  while (home_switch != HIGH) // When the switch is not on 
  {
      digitalWrite(dirPin, HIGH);    
                        //trying to find a way where it can detect where home is 
  }
}

void Move_home1()
  {
   for(int x = 0; x < stepsPerRevolution; x++)
   {
      while(home_switch = 0 && position ==0)
      {
      currentMotorStep ++; //increments 
      position = position +1; 
      }
      while(home_switch = 1 && position != 0) //if switch is on and position is not at zero then it will decrement to home 
      {
        currentMotorStep--; 
        position = position -1; 
      }
   }
  }
