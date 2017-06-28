#include "oAvoider.h"

ObstacleAvoider::ObstacleAvoider(int newSensorPins[NUM_SENSORS][2], int distanceToStartAvoiding, int closestAllowableDistance)          
{
  memcpy(sensorPins, newSensorPins, sizeof(sensorPins));
  this->distanceToStartAvoiding = distanceToStartAvoiding;
  this->closestAllowableDistance = closestAllowableDistance;
  for(int sensorPin = 0; sensorPin < NUM_SENSORS; sensorPin++) {pinMode(sensorPins[sensorPin][0], OUTPUT); pinMode(sensorPins[sensorPin][1], INPUT);}
}

float ObstacleAvoider::calcObstacleAvoidanceForce(int sensorNum)
{ 
  float distanceToNearestObject = getNearestObjectProximity(sensorNum);
  if(distanceToNearestObject > distanceToStartAvoiding) {return 0;}
  float adjustmentForce = objectDistanceToAdjustmentForce(distanceToNearestObject);
  return regulateAdjustmentForce(adjustmentForce);
}

float ObstacleAvoider::objectDistanceToAdjustmentForce(float distanceToObject)   
{
 return map(distanceToObject, distanceToStartAvoiding, closestAllowableDistance, 0, maximumAdjustmentForce);                                                                                                                                                              
}

float ObstacleAvoider::regulateAdjustmentForce(float adjustmentForce)
{
  if(adjustmentForce > maximumAdjustmentForce) {return maximumAdjustmentForce;}  
  return adjustmentForce;
}

float ObstacleAvoider::getNearestObjectProximity(int sensorNum)
{
  int reading = getSensorReading(sensorNum);
  return sensorReadingToDistance(reading);
}

float ObstacleAvoider::sensorReadingToDistance(int sensorReading) {return sensorReading*0.034/2;}   //uses speed of sound in room temperature

int ObstacleAvoider::getSensorReading(int sensorNum)  //returns the sensor reading, check out the ultrasonic sensor example for arduino for explanations on how to use the sensors
{
  digitalWrite(sensorPins[sensorNum][0], HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorPins[sensorNum][0], LOW);
  return pulseIn(sensorPins[sensorNum][1], HIGH);
}



float *ObstacleAvoider::allSensorReadingsToDistances(int readings[NUM_SENSORS])                                
{
  float distances[NUM_SENSORS];
  for(int sensorNum = 0; sensorNum < NUM_SENSORS; sensorNum++) {distances[sensorNum] = sensorReadingToDistance(readings[sensorNum]);}
  return distances;
}

int *ObstacleAvoider::getAllSensorReadings()       
{
  int readings[NUM_SENSORS];
  for(int sensorNum = 0; sensorNum < NUM_SENSORS; sensorNum++) {readings[sensorNum] = getSensorReading(sensorNum);}
  return readings;
}



int ObstacleAvoider::getClosestAllowableDistance(){return closestAllowableDistance;}                                                      
void ObstacleAvoider::setClosestAllowableDistance(int newDistance) {this->closestAllowableDistance = newDistance;}

int ObstacleAvoider::getDistanceToStartAvoiding(){return distanceToStartAvoiding;}
void ObstacleAvoider::setDistanceToStartAvoiding(int newDistance) {this->distanceToStartAvoiding = newDistance;}

void ObstacleAvoider::setSensorPins(int newPins[NUM_SENSORS][2]) {memcpy(this->sensorPins, newPins, sizeof(newPins));}   
