#include "MotorController.h"

MotorController::MotorController(int motorPins[4], int motorSpeedOffsets[4] = {})                 //takes in the motorpins and offsets
{
  memcpy(this->motorPins, motorPins, sizeof(motorPins));
  memcpy(this->motorSpeedOffsets, motorSpeedOffsets, sizeof(motorSpeedOffsets));
  for(int motorNum = 0; motorNum < NUM_MOTORS; motorNum++) {motors[motorNum].attach(motorPins[motorNum]); currentMotorSpeeds[motorNum] = 0;} 
}

void MotorController::updateMotorSpeeds(int newMotorSpeeds[NUM_MOTORS])
{
  for(int motorNum = 0; motorNum < NUM_MOTORS; motorNum++) {updateMotorSpeed(motorNum, newMotorSpeeds[motorNum]);}
}

void MotorController::updateMotorSpeed(int motorNum, int newMotorSpeed)
{
  currentMotorSpeeds[motorNum] = limitMotorSpeed(newMotorSpeed);
  writeMotorNewSpeed(motorNum, currentMotorSpeeds[motorNum]);
}




void MotorController::incrementAndWriteMotorSpeeds(int increments[])                                                     
{
   for(int motorNum = 0; motorNum < NUM_MOTORS; motorNum++) 
   {incrementAndWriteMotorSpeed(motorNum, increments[motorNum]);}
}

void MotorController::incrementAndWriteMotorSpeed(int motorNumber, int increment = 1) 
{
  currentMotorSpeeds[motorNumber] = limitMotorSpeed(currentMotorSpeeds[motorNumber]);
  writeMotorNewSpeed(motorNumber, currentMotorSpeeds[motorNumber]);
} 


int *MotorController::limitMotorSpeeds(int motorSpeeds[NUM_MOTORS])
{
  int limitedSpeeds[NUM_MOTORS];
  for(int motorNum = 0; motorNum < NUM_MOTORS; motorNum++) {limitedSpeeds[motorNum] = limitMotorSpeed(motorSpeeds[motorNum]);}
  return limitedSpeeds;
}

int MotorController::limitMotorSpeed(int motorSpeed)
{
  if(motorSpeed > MAX_MOTOR_SPEED) {return MAX_MOTOR_SPEED;}
  else if(motorSpeed < MIN_MOTOR_SPEED) {return MIN_MOTOR_SPEED;}
  return motorSpeed;
}



void MotorController::writeMotorsNewSpeeds(int newMotorSpeeds[])                                                  
{
  for(int motorNum = 0; motorNum < NUM_MOTORS; motorNum++) {writeMotorNewSpeed(motorNum, newMotorSpeeds[motorNum]);}
}

void MotorController::writeMotorNewSpeed(int motorNumber, int newSpeed)                                                                         
{               
  motors[motorNumber].writeMicroseconds(offsetMotorSpeed(motorNumber, newSpeed));                     
}

int *MotorController::offsetMotorSpeeds(int motorSpeeds[NUM_MOTORS]) 
{
  int offsetMotorSpeeds[NUM_MOTORS];
  for(int motorNum = 0; motorNum < NUM_MOTORS; motorNum++) {offsetMotorSpeeds[motorNum] = offsetMotorSpeed(motorNum, motorSpeeds[motorNum]);} 
  return offsetMotorSpeeds; 
}

int MotorController::offsetMotorSpeed(int motorNum, int motorSpeed) {return motorSpeed - motorSpeedOffsets[motorNum];}



void MotorController::applyStateAdjustmentForceToMotors(int axisNum, float adjustmentForce)
{
  int *speedAdjustments = convertAdjustmentForceToSpeedAdjustments(adjustmentForce);
  int *directionalConstants = calcMotorSpeedDirectionConstants(axisNum);                                   //these determines the direction of the speed adjustment
  for(int motorNum = 0; motorNum < NUM_MOTORS; motorNum++)
  {
    speedAdjustments[motorNum] = speedAdjustments[motorNum] * directionalConstants[motorNum] * .5;        //the .5 is because the motors are split into two groups due to the directional constants, each bearing half the load
  }
  incrementAndWriteMotorSpeeds(speedAdjustments);
}

int *MotorController::convertAdjustmentForceToSpeedAdjustments(float adjustmentForce)
{
  int motorSpeedAdjustments[NUM_MOTORS];
  for(int motorNum = 0; motorNum < NUM_MOTORS; motorNum)
  {
   motorSpeedAdjustments[motorNum] = adjustmentForce - currentMotorSpeeds[motorNum];
  }
  return motorSpeedAdjustments;
}

int MotorController::calcMotorSpeedDirectionConstants(int axisNum)        //here we determine which direction the speed adjustment should go for each motor
{
  switch(axisNum)                                                  
  {
    case 0:                                                                         
    return calcPitchDirectionConstants();  
    break;
    
    case 1:                              
    return calcRollDirectionConstants();                                               
    break;
    
    case 2:                            
    return calcYawDirectionConstants();                 
    break;
    
    case 3:                            
    return calcAltitudeDirectionConstants();                                                
    break;
  }
}

int *MotorController::calcPitchDirectionConstants()
{
  int directionConstants[NUM_MOTORS];
  for(int motorNum = 0; motorNum < NUM_MOTORS / 2; motorNum++) {directionConstants[motorNum] = -1;} //front motors go one way
  for(int motorNum = NUM_MOTORS / 2; motorNum < NUM_MOTORS; motorNum++) {directionConstants[motorNum] = 1;} //back motors do the opposite
  return directionConstants;                                                                        //results in a net movement forward or back
}

int *MotorController::calcRollDirectionConstants()
{
  int directionConstants[NUM_MOTORS];
  for(int motorNum; motorNum < NUM_MOTORS; motorNum++) {directionConstants[motorNum] = 1;} //just initialzing the array
  directionConstants[0] = 1;                    //first and last motor are on the left side, they go one way
  directionConstants[NUM_MOTORS - 1] = 1;
  directionConstants[NUM_MOTORS / 2] = -1;      //these are the two motors on the right side, they go the other
  directionConstants[NUM_MOTORS / 2 - 1] = -1;
  return directionConstants;                    //this results in a net movement to the right or left
}

int *MotorController::calcYawDirectionConstants()
{
  int directionConstants[NUM_MOTORS];
  for(int motorNum = 0; motorNum < NUM_MOTORS; motorNum++) 
  {
    if(motorNum % 2 == 0) {directionConstants[motorNum]  = 1;}      //all clockwise either slow or speed up
    else{directionConstants[motorNum] = -1;}                        //all counter clockwise do the opposite
  }
  return directionConstants;                                        //this results in a rotation of the drone body
}

int *MotorController::calcAltitudeDirectionConstants()
{
  int directionConstants[NUM_MOTORS];
  for(int motorNum = 0; motorNum < NUM_MOTORS; motorNum++) {directionConstants[motorNum] = 1;}  //all motors act together
  return directionConstants;
}



void MotorController::motorTest()                                                                                                               
{
  for(int i = 0; i < 1000; i += 10) {int testSpeeds[4] = {1000 + i, 1000 + i, 1000 + i, 1000 + i}; writeMotorsNewSpeeds(testSpeeds); delay(100);}
  for(int i = 1000; i > 0; i -= 10) {int testSpeeds[4] = {1000 + i, 1000 + i, 1000 + i, 1000 + i}; writeMotorsNewSpeeds(testSpeeds); delay(100);}
}

//getters and setters below here
int *MotorController::getMotorSpeedOffsets() {return motorSpeedOffsets;}
void MotorController::setMotorSpeedOffsets(int newOffsets[NUM_MOTORS]) {memcpy(motorSpeedOffsets, newOffsets, sizeof(newOffsets));}
int *MotorController::getMotorPins() {return motorPins;}
void MotorController::setMotorPins(int newMotorPins[NUM_MOTORS]) 
{
  memcpy(motorPins, newMotorPins, sizeof(motorPins)); 
  for(int motorNum = 0; motorNum < NUM_MOTORS; motorNum++) 
  { 
    motors[motorNum].attach(newMotorPins[motorNum]); 
    writeMotorsNewSpeeds(currentMotorSpeeds);
  }
}

