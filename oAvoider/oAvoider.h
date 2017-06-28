#ifndef oAvoider_h
#define oAvoider_h

#include "Arduino.h"

class ObstacleAvoider
{

  friend class PIDcontroller;     //the PID controller will have special access in order to adjust paramters in its avoider object


  public:
    static const int NUM_SENSORS = 4;
    static const float maximumAdjustmentForce = 250;
    
    ObstacleAvoider(int newSensorPins[NUM_SENSORS][2], int distanceToStartAvoiding, int closestAllowableDistance); 
    float calcObstacleAvoidanceForce(int sensorNum);                            
    float objectDistanceToAdjustmentForce(float distanceToObject);              
    float regulateAdjustmentForce(float adjustmentForce);

    int getSensorReading(int sensorNum);                                                                     
    int *getAllSensorReadings();                                                                         
    float sensorReadingToDistance(int sensorReading);                                                              
    float *allSensorReadingsToDistances(int sensorReadings[NUM_SENSORS]);                                   
    float getNearestObjectProximity(int sensorNum);

    //getters and setters
    void setClosestAllowableDistance(int newDistance);
    int getClosestAllowableDistance();
    void setDistanceToStartAvoiding(int newDistance);
    int getDistanceToStartAvoiding();
    void setSensorPins(int newPins[NUM_SENSORS][2]);
    
  private:
    int sensorPins[NUM_SENSORS][2];                                                               //ultrasonic sensor pins
    int closestAllowableDistance;                                                                              
    int distanceToStartAvoiding;                                                                              
    
};

#endif
