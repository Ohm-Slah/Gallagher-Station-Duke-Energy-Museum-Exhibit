/* Wire Controller Reader
   Arduino code:
   address is 0x40
   SDA is 20
   SCL is 21

   This code pings the slave device to receive data and when the data is received successfully,
   send a ping back to the master to print a statement. refference slaveI2C for rest of code.

*/
#include <Wire.h>

int output = 1234; 

void setup()
{
  Wire.begin(0x40);        // join i2c bus (address optional for master)
  Serial.begin(9600);
}
void loop()
{
   byte iiii = (output/1000) % 10; // == 1
   byte  iii = (output/100) % 10;  // == 2
   byte   ii = (output/10) % 10;   // == 3
   byte    i = output % 10;        // == 4
   Wire.beginTransmission(8); //transmit to device 
   Wire.write(iiii);   // sends one byte
   Wire.write(iii);
   Wire.write(ii);
   Wire.write(i);
   Wire.endTransmission();   //stop transmitting
   Serial.println(output);
   Serial.println(iiii);
   Serial.println(iii);
   Serial.println(ii);
   Serial.println(i); 
   output++; 
   delay(500);
  if(output > 9999){
   output = 0001;
  }

}
