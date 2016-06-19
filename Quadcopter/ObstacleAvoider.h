#ifndef ObstacleAvoider_h
#define ObstacleAvoider_h

#include "Arduino.h"

class ObstacleAvoider
{

  friend class Quad; //allows Quad to access private variables

  public:
  ObstacleAvoider(int p_sensorPins[]);

  void obstacleAvoidance(int axis);

  int getSensorReading(int sensorNum);
  int convertReadtoDistance(int reading);
  void getAllSensorReadings();

  ~ObstacleAvoider();
  
  private:
  int sensorReadings[4]; //holds the readings converted to distance 
  int sensorPins[4]; //keeps track of the four ir sensors used for obstacle avoidance

  int DISTANCE_THRESHOLD = 15;

  bool sensorMoving = {false, false, false}; //same thing as above but for obstacle avoidance
  
};


#endif
