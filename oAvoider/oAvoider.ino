#include "oAvoider.h"

int minDistance = 35;
int maxDistance = 10;

int sensorPins[4][2] = {{11, 10}, {9, 8}, {5,6}, {4,3}};

int pro = 100;
int integral = 150;
int deriv = -100;

ObstacleAvoider newAvoider(sensorPins, minDistance, maxDistance);
void setup() 
{
  Serial.begin(9600);
}

void loop() 
{
  
  for(int i = 0; i < 4; i++)
  {
     Serial.println("Axis: " + String(i));
     Serial.println("Reading: " + String(newAvoider.getReading(i)));
     Serial.println("Distance: " + String(newAvoider.getDistance(newAvoider.getReading(i))));
     Serial.println("Avoidance Adjust: " + String(newAvoider.obstacleAvoidAxis(i, pro, integral, deriv)));
     Serial.println();
  }
  delay(1000);
  Serial.println("---------------------\n\n");
}
