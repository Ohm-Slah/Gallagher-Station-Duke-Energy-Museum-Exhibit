/* Wire Controller Reader
 * Arduino code:
 * address is 0x40
 * SDA is 20
 * SCL is 21
 * 
 * This code pings the slave device to receive data and when the data is received successfully, 
 * send a ping back to the master to print a statement. refference slaveI2C for rest of code. 
 * 
 */


#define F_CPU 8000000

#include <Wire.h>

void setup() {

  Wire.begin(0x40);        // join i2c bus (address optional for master)

  Serial.begin(9600);  // start serial for output

}


void loop() {

  Wire.requestFrom(8, 6);    // request 6 bytes from peripheral device #8


  while (Wire.available()) { // peripheral may send less than requested

    char c = Wire.read(); // receive a byte as character

    Serial.print(c);         // print the character

  }


  delay(500);

}
