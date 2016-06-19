#include "Balancer.h"


Balancer::Balancer(int p_accelerometerPins[])
{

  memcpy(accelerometerPins, p_accelerometerPins, sizeof(p_accelerometerPins));
  
}

int Balancer::readAccelerometerPin(int numAccel) {return analogRead(accelerometerPins[numAccel]);}
int Balancer::readAllAccelerometerPins()
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

Balancer::~Balancer() {delete[] accelerometeReadings; delete[] accelerometerPins;}
