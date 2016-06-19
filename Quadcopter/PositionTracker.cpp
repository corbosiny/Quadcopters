#include "PositionTracker.h"

#include <Wire.h> //accelerometer libraries(MPU 6050)
#include <MPU6050.h> //part of MPU 6050
#include <I2Cdev.h> //part of MPU 6050

PositionTracker::PositionTracker()
{

  MPU6050 accelgyro; //defines name of accelerometer
  tolerances = {40, 40, 40};
  
}

PositionTracker::PositionTracker(int p_tolerances)
{

  MPU6050 accelgyro;
  memcpy(tolerances, p_tolerances, sizeof(p_tolerances));
  
}

float PositionTracker::calcTimeInterval(float startTime) {return (millis() - startTime);}
float PositionTracker::calcAcceleration(int axis) 
{

  int mpuReadings[3];
  accelgyro.getAcceleration(mpuReadings[0], mpuReadings[1], mpuReadings[2]);;
  return mpuReadings[axis - 1];

}
float PositionTracker::calcVelocity(float axisAcceleration, float timeInterval) {return (axisAcceleration * timeInterval)}
float PositionTracker::calcPosition(int axis, float velocity, float velocityChange, float timeInterval) {return ((velocity + (velocity - velocityChange)) / 2) * timeInterval;}  

void PositionTracker::allignAxis(int axis, int num = 0)
{

  if(!moving[axis - 1])
  {
    
    if(positions[axis - 1] < userPositions[axis - 1]) {moveAxis(0 - axis);}
    else if(positions[axis - 1] > userPositions[axis - 1]) {moveAxis(axis);}

  }

  if(num == 0 && movingAxis[axis - 1])
  {

    if(positions[abs(axis) - 1] < userPositions[abs(axis) - 1] + tolerances[abs(axis) - 1] || positions[abs(axis) - 1] > userPositions[abs(axis) - 1] - tolerances[abs(axis) - 1])
    {

      moveAxis(axis, 1);
    
    }

  }
  else if(movingAxis[axis - 1])
  {

    if(positions[abs(axis) - 1] < userPositions[abs(axis) - 1] + 5 || positions[abs(axis) - 1] > userPositions[abs(axis) - 1] - 5)
    {

      moveAxis(axis, 1);
     
    }

  }
  
}

void PositionTracker::allignAllAxises(int num = 0) 
{

  for(int i = 0; i < 3; i++) {allignAxis(i, num);}
  
}

void PositionTracker::follow() {allignAllAxises(0);}

void PositionTracker::headToPoint(int coordinates) {allignAllAxises(1);}

void PositionTracker::returnHome() {headToPoint(padPosition);}

PositionTracker::~PositionTracker() {delete startingTime; delete initialPosition; delete movingAxis[]; delete[] velocities; delete[] positions; delete[] userPositions; delete[] tolerances; delete[] padPosition;}
