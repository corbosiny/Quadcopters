#include "oAvoider.h"

const int numSensors = 4;

int minDistance = 35;
int maxDistance = 10;

int sensorPins[4][2] = {{11, 10}, {9, 8}, {5,6}, {4,3}};

float maxAdjustmentForce = 250;

ObstacleAvoider newAvoider(sensorPins, minDistance, maxDistance);
void setup() 
{
  Serial.begin(9600);
//  ObstacleAvoider.maxAdjustmentForce = maxAdjustmentForce;
}

void loop() 
{
  
  for(int i = 0; i < numSensors; i++)
  {
     Serial.println("Axis: " + String(i));
     Serial.println("Reading: " + String(newAvoider.getSensorReading(i)));
     Serial.println("Distance: " + String(newAvoider.sensorReadingToDistance(newAvoider.getSensorReading(i))));
     Serial.println("Avoidance Adjust: " + String(newAvoider.calcObstacleAvoidanceForce(i)));
     Serial.println();
  }
  delay(1000);
  Serial.println("---------------------\n\n");
}
