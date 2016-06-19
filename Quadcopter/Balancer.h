#ifndef Balancer_h
#define Balancer_h

#include "Arduino.h"

class Balancer
{

  friend class Quad; //allows Quad to access private variables
  
  public:
  Balancer(int p_accelerometerPins);

  int readAccelerometerPin(int numAccel);
  int readAllAccelerometerPins();

  void selfBalance();

   ~Balancer(); //Deconstructor

  private:
  int accelerometerReadings[4];
  int accelerometerPins[4];
  
};

#endif
