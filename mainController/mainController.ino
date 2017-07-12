//Feromone Robotics Qaudcopter main flight controller
//This controller is responsible for calculating the change of states needed in order to
//fly toward a certain point and feeds those states to the PIDS
//Developers: Corey Hulse
//***See flight system manual for a description of all the other systems***
//GPS = Adafruit Ultimate GPS module
//rxPin and txPin to their respecitve pins on the GPS
//Vin - 3 to 5 volts
//GND - GND
//Don't worry about the rest of the pins on the GPS

int rxPin, txPin;

#include "PID.h"
#include "IMU.h"
#include "MotorController.h"
#include "oAvoider.h"
#include "GPS.h"

int motors[] = {1,2,3,4};
int constants[] = {5,5,.4};
int maxOutputs[] = {255, 255, 255};
int maxAdjust = 45;  //degrees
int maxDistance = 30;
int minDistance = 15;
int sensorPins[4][2] = {{11,10}, {9,8}, {7,6}, {5,4}};
int distanceTolerance = 3;
int destinationPoint[2];
//building up and initizilzing our drone systems
IMU newIMU;
GPS adaGPS(rxPin, txPin); 
MotorController motorC(motors);
ObstacleAvoider oAvoider(sensorPins, maxDistance, minDistance);
PIDcontroller pids(&newIMU, constants, maxOutputs);

void setup() 
{
  destinationPoint[0] = adaGPS.getLatitude();
  destinationPoint[1] = adaGPS.getLongitude();
}

void loop() 
{

  pids.updateStateAdjustmentsToReachDesiredStates();
  adaGPS.updateGPSData();                                                  
  float headingDiff = calcRelationHeading(destinationPoint);        //polar coordinates to our desired location
  if(headingDiff != -1)
  {
    float newPitchTarget = sin(headingDiff) * maxAdjust;           //breaking up our movements in x and y into pitch and roll movements
    float newRollTarget = cos(headingDiff) * maxAdjust;

    pids.setDesiredState(0, newPitchTarget);                          //setting our PID controller to put the body into that position so it will move how we want it
    pids.setDesiredState(1, newRollTarget);
  }
  
}


float calcRelationHeading(int point[2]) //calculates the polar coordinates to get to a desired point, 0 index = lattitude, 1 index = longitude
{

  int latDifference = point[0] - adaGPS.getLatitude(); 
  int lonDifference = point[1] - adaGPS.getLongitude();
  if(latDifference < distanceTolerance && lonDifference < distanceTolerance) {return -1;}
  int distanceMag = sqrt(sq(latDifference) + sq(lonDifference));
  int angleRelation = atan(lonDifference/latDifference);
  angleRelation *= 180 / PI;
  if(angleRelation < 0 && latDifference < 0) {angleRelation += 180;} 
  return angleRelation;

}

