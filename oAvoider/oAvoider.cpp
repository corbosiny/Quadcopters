#include "oAvoider.h"

ObstacleAvoider::ObstacleAvoider(int newPins[4][2], int mxDistance, int mnDistance)                             //takes in the four IR sensors and the distance range it should start avoiding on
{
  memcpy(sensorPins, newPins, sizeof(sensorPins));
  this->maxDistance = mxDistance;
  this->minDistance = mnDistance;
  for(int i = 0; i < 4; i++) {pinMode(sensorPins[i][0], OUTPUT); pinMode(sensorPins[i][1], INPUT);}
}

int ObstacleAvoider::getMinDistance(){return minDistance;}                                                      
void ObstacleAvoider::setMinDistance(int newMin) {this->minDistance = newMin;}

int ObstacleAvoider::getMaxDistance(){return maxDistance;}
void ObstacleAvoider::setMaxDistance(int newMax) {this->maxDistance = newMax;}

void ObstacleAvoider::setSensorPins(int newPins[4][2]) {memcpy(this->sensorPins, newPins, sizeof(newPins));}   

int ObstacleAvoider::getReading(int axis)  //returns the sensor reading, check out the ultrasonic sensor example for arduino for explanations on how to use the sensors
{
  digitalWrite(sensorPins[axis][0], LOW);
  delayMicroseconds(2);
  digitalWrite(sensorPins[axis][0], HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorPins[axis][0], LOW);
  return pulseIn(sensorPins[axis][1], HIGH);
}

int *ObstacleAvoider::getReadings()       //returns readings from every axis
{
  int readings[4];
  for(int i = 0; i < 4; i++) {readings[i] = getReading(i);}
  return readings;
}

float ObstacleAvoider::getDistance(int reading) {return reading*0.034/2;}   //converts a reading to distance from the nearest object, uses speed of sound in room temp air. May incorporate a temperature sensor

float *ObstacleAvoider::getDistances(int readings[])                        //returns the proximity on each axis to the nearest axis
{
  float distances[4];
  for(int i = 0; i < 4; i++) {distances[i] = getDistance(readings[i]);}
  return distances;
}

float ObstacleAvoider::obstacleAvoidAxis(int axis, float proportional, float integral, float derivative)      //calculates adjustment forces on an axis based on object proximity
{
 int reading = getReading(axis);
 float distance = getDistance(reading);                                                                       //getting object proximity
 if(distance == 0 || distance > maxDistance) {return 0;}                                                      //if the object is outside the range we want to start avoiding at, then just return no adjust
 float tempForce = proportional + integral + derivative;                                                      //maximum force we want output for our adjustments
 float mapped = map(distance, maxDistance, minDistance, 0, tempForce);                                        //calculates adjustment force
 if(mapped == NAN) {return 0;}                                                                                //catches any weird values when we are on the edge of our ranges
 else if(mapped > tempForce) {return tempForce;}                                                              //limits our adjustment force output
 else if(mapped < 0) {return 0;}                                                                              //still catching weird values we might get                                  
 return mapped;
}

float *ObstacleAvoider::obstacleAvoid(float proportional[4], float integral[4], float derivative[4])      
{
  float adjusts[4];
  for(int i = 0; i < 4; i++) {adjusts[i] = obstacleAvoidAxis(i, proportional[i], integral[4], derivative[i]);}
  return adjusts;
}

