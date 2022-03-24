/*
 * ya yeet
 * This code is for the slave device to receive 'n' bits of data from an i2c bus defined by 
 * the master device.
 * Data requests happen every second and print hello to serial monitor
 * see file masterI2C for rest of code
 */


#include <Wire.h>


void setup() {

  Wire.begin(8);                // join i2c bus with address #8

  Wire.onRequest(requestEvent); // register event

}


void loop() {

  delay(100);

}


// function that executes whenever data is requested by master

// this function is registered as an event, see setup()

void requestEvent() {

  Wire.write("hello "); // respond with message of 6 bytes

  // as expected by master

}
