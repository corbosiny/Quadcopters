#include "ObstacleAvoider.h"

ObstacleAvoider::ObstacleAvoider(int p_sensorPins[])
{

  memcpy(sensorPins, p_sensorPins, sizeof(p_sensorPins));
  
}

int ObstacleAvoider::getSensorReading(int sensorNum) {return analogRead(sensorPins[sensorNum - 1]);}
int ObstacleAvoider::convertReadtoDistance(int reading) {return (6787 / (reading - 3)) - 4;}

void ObstacleAvoider::setDistanceThreshold(int newThresh) {DISTANCE_THRESHOLD = newThresh;}
int ObstacleAvoider::getDistanceThreshold() {return DISTANCE_THRESHOLD;}

void ObstacleAvoider::obstacleAvoidance(int axis)
{

 if(sensorReadings[0] <= DISTANCE_THRESHOLD && sensorReadings[0] != 0) {return 1;} //will be piped to move axis as the returned number comma 1 for all of these return statements
 if(sensorReadings[1] <= DISTANCE_THRESHOLD && sensorReadings[0] != 0) {return 2;}
 if(sensorReadings[2] <= DISTANCE_THRESHOLD && sensorReadings[2] != 0) {return -1;}
 if(sensorReadings[3] <= DISTANCE_THRESHOLD && sensorReadings[3] != 0) {return -2;}
  
}



void ObstacleAvoider::getAllSensorReadings()
{

  for (int i = 0; i < sizeof(sensorPins)/sizeof(sensorPins[0]); i++)
  {

    int distance = convertReadtoDistance(getSensorReading(i));
    sensorReadings[i] = distance;
    
  }
  
}

ObstacleAvoider::~ObstacleAvoider() {delete sensorReading; delete[] sensorPins; delete DISTANCE_THRESHOLD; delete[] sensorMoving;}
