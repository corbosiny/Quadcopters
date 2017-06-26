#ifndef oAvoider_h
#define oAvoider_h

#include "Arduino.h"

class ObstacleAvoider
{

  friend class PIDcontroller;                                                                     //the PID controller will have special access in order to adjust paramters in its avoider object

  public:
    ObstacleAvoider(int newPins[4][2], int maxDistance, int minDistance);                         //just needs sensor pins, a proximity to start avoiding at, and minimum proximity allowed at            
    float obstacleAvoidAxis(int axis, float proportional, float integral, float derivative);      //needs the axis and the parts to find the value of the 
    float *obstacleAvoid(float proportional[4], float integral[4], float derivative[4]);          //returns adjustment for every axis

    //getters and setters
    void setMinDistance(int newMin);
    int getMinDistance();
    void setMaxDistance(int newMax);
    int getMaxDistance();
    void setSensorPins(int newPins[4][2]);

    int getReading(int axis);                                                                     //get a sensor reading on that axis
    int *getReadings();                                                                           //gets a sensor reading on each axis
    float getDistance(int reading);                                                               //gets distance to nearest object on a direction
    float *getDistances(int readings[]);                                                          //gets distance to nearest object on each direction
    
  private:
    int sensorPins[4][2];                                                                         //ultrasonic sensor pins
    int minDistance;                                                                              //the closest allowable distance to an object(maximum adjust forces will be applied)
    int maxDistance;                                                                              //proximity we start avoiding at
    
};

#endif
