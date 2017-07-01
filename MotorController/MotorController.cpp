#include "MotorController.h"

MotorController::MotorController(int p_motorPins[4], int p_motorSpeedOffsets[4] = {})                 //takes in the motorpins and offsets
{
  memcpy(motorPins, p_motorPins, sizeof(p_motorPins));
  memcpy(motorSpeedOffsets, p_motorSpeedOffsets, sizeof(p_motorSpeedOffsets));
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

void MotorController::incrementAndWriteMotorSpeeds(float increments[])                                                     
{
   for(int motorNum = 0; motorNum < NUM_MOTORS; motorNum++) 
   {incrementAndWriteMotorSpeed(motorNum, increments[motorNum]);}
}

void MotorController::incrementAndWriteMotorSpeed(int motorNumber, int increment = 1) 
{
  currentMotorSpeeds[motorNumber] = limitMotorSpeed(currentMotorSpeeds[motorNumber]);
  writeMotorNewSpeed(motorNumber, currentMotorSpeeds[motorNumber]);
} 

int *MotorController::offsetMotorSpeeds(int motorSpeeds[NUM_MOTORS]) 
{
  int offsetMotorSpeeds[NUM_MOTORS];
  for(int motorNum = 0; motorNum < NUM_MOTORS; motorNum++) {offsetMotorSpeeds[motorNum] = offsetMotorSpeed(motorNum, motorSpeeds[motorNum]);} 
  return offsetMotorSpeeds; 
}

int MotorController::offsetMotorSpeed(int motorNum, int motorSpeed) {return motorSpeed - motorSpeedOffsets[motorNum];}

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

