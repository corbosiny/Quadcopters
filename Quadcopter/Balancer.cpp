#include "Balancer.h"


Balancer::Balancer(int p_accelerometerPins[])
{

  memcpy(accelerometerPins, p_accelerometerPins, sizeof(p_accelerometerPins)); //makes an array of accelerometerPins based off the entered list
  
}

int Balancer::readAccelerometerPin(int numAccel) {return analogRead(accelerometerPins[numAccel]);} //returns the acceleration on one of the accelerometer pins

int Balancer::readAllAccelerometerPins() //reads each accelereometer pin and stores its reading int he respective index over in accelerometer readings
{

  for(int i = 0; i < sizeof(accelerometerReadings) / sizeof(accelerometerReadings[0]); i++) {accelerometerReadings[i] = readAccelerometerPin(i);} 
  
}

void Balancer::selfBalance()
{
  
  for(int i = 0; i < sizeof(accelerometerReadings) / sizeof(accelerometerReadings[0]; i++)
  {

    if(accelerometerReadings[i] != 0) {return accelerometerReadings[i];} //adds it to motorSpeeds
    
  }
  
}

Balancer::~Balancer() {delete[] accelerometeReadings; free(accelerometerPins);} //deconstructor for the class to free
