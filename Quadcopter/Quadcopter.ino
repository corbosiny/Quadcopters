//check Quad.h for documentation and code explanations

#include "Quad.h" 

void setup() 
{

  int motorPins[4] = {0,0,0,0}; //change all these arrays with the right pins
  int accelerometerPins[4] = {0,0,0,0}; 
  int sensorPins[4] = {0,0,0,0};
  Quad testCopter(motorPins, accelerometerPins, sensorPins); //read documentation for the right paramters you should use

  testCopter.takeOff(); //feel free to change all this for testing purposes
  delay(5000);
  testCopter.land();
  delay(5000);
  testCopter.takeOff();
  delay(3000);
  testCopter.moveAxis(1);
  delay(1000);
  testCopter.moveAxis(1);
  delay(1000);
  testCopter.moveAxis(-1);
  delay(1000);
  testCopter.moveAxis(-1);
  testCopter.land();

}

void loop() {

  
  
}
