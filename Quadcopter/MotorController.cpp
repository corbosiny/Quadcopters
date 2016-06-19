#include "MotorController.h"

MotorController::MotorController(int p_motorPins[])
{

  memcpy(motorPins, p_motorPins, sizeof(p_motorPins));
  
}

void MotorController::writeAllMotors(bool state)
{

  for(int i = 0; i < 4; i++) {digitalWrite(motorPins[i], state);}
  
}
void MotorController::writeAllMotors(int pwmSignal)
{

  for(int i = 0; i < sizeof(motorPins) / sizeof(motorPins[0]; i++) {digitalWrite(motorPins[i], pwmSignal);}
  
}

void MotorController::writeAllMotors(int speeds[])
{

  for(int i = 0; i < sizeof(speeds)/sizeof(speeds[0]))
  {

    analogWrite(motorPins[i], speeds[i]);
    
  }

}

void MotorController::writeMotor(int motorNum, bool state) {digitalWrite(motorPins[motorNum], state);}
void MotorController::writeMotor(int motorNum, int pwmSignal) {digitalWrite(motorPins[motorNum], pwmSignal);}

void MotorController::moveAxis(int axis, int undoing = 0) //only inputs piped from obstacle avoider change the undoing value
{

  if(undoing && abs(axis) != 3) 
  {
      
      moving[axis - 1] = false;
      
      motorSpeeds[axis - 1] = CRUISING_SPEED;
      
      motorSpeeds[axis] = CRUISING_SPEED;
  
  }
  else if(undoing && abs(axis) == 3)
  {

    moving[axis - 1] = false;
    for(int i = 0; i <4; i++) {motorSpeeds[i] = CRUISING_SPEED;}
    
  }
  else if(abs(axis) != 3)
  {
    
      moving[axis - 1] = true;

      if(axis > 0)
      {
      
        if(motorSpeeds < CRUISING_SPEED) {motorSpeeds[axis - 1] = CRUSING_SPEED;}
        if(motorSpeeds < CRUISING_SPEED) {motorSpeeds[axis] = CRUISING_SPEED;}
      
      }
      else if(axis < 0)
      {

        if(motorSpeeds > CRUISING_SPEED) {motorSpeeds[axis - 1] = CRUISING_SPEED;}
        if(motorSpeeds > CRUISING_SPEED) {motorSpeeds[axis] = CRUISING_SPEED;}
        
      }
      
      if(motorSpeeds[axis - 1] + adjustmentSpeeds[0] * (abs(axis) / axis) > MAX_SPEED) {motorSpeeds[axis - 1] = MAX_SPEED;}
      else if(motorSpeeds[axis - 1] + adjustmentSpeeds[0] * (abs(axis) / axis) < 0) {motorSpeeds[axis - 1] = 0;}
      else {motorSpeeds[axis - 1] += adjustmentSpeeds[0] * (abs(axis) / axis);}
      
      if(motorSpeeds[axis] + adjustmentSpeeds[0] * (abs(axis) / axis) > MAX_SPEED) {motorSpeeds[axis] = MAX_SPEED;}
      else if(motorSpeeds[axis] + adjustmentSpeeds[0] * (abs(axis) / axis) < 0) {motorSpeeds[axis] = 0;}
      else {motorSpeeds[axis] += adjustmentSpeeds[0] * (abs(axis) / axis);} 
  
  }
  else if(abs(axis) == 3)
  {

    moving[axis - 1] = true;
    for(int i = 0; i < 4; i++) {motorSpeeds[i] += adjustmentSpeeds[0];}
    
  }

  for(int i = 0; i < 4; i++)
  {

    if(motorSpeeds[i] < 0) {motorSpeeds[i] = 0;}
    else if(motorSpeeds[i] > MAX_SPEED) {motorSpeeds[i] = MAX_SPEED;}
    
  }
  
}

void MotorController::moveAxisSensor(int axis, int undoing)
{

  if(undoing && sensorMoving[abs(axis) - 1]) 
  {
    
    sensorMoving[abs(axis) - 1] = !sensorMoving[abs(axis) - 1];
    
    motorSpeeds[abs(axis) - 1] -= adjustSpeeds[1] * (abs(axis) / axis);
    motorSpeeds[abs(axis)] -= adjustSpeeds[1] * (abs(axis) / axis);
      
  }
  else
  {

    if(axis > 0 && sensorMoving[axis]) 
    {

      if(motorSpeeds[axis - 1] < CRUISING_SPEED) {motorSpeeds[axis - 1] += adjustSpeeds[1];}
      if(motorSpeeds[axis] < CRUISING_SPEED) {motorSpeeds[axis] += adjustSpeeds[1];}
    
    }

    if(axis < 0 && sensorMoving[axis]) 
    {

      if(motorSpeeds[axis - 1] > CRUISING_SPEED) {motorSpeeds[axis - 1] -= adjustSpeeds[1];}
      if(motorSpeeds[axis] > CRUISING_SPEED) {motorSpeeds[axis] -= adjustSpeeds[1];}
      
    }
    
    if(motorSpeeds[axis - 1] + adjustmentSpeeds[1] * (abs(axis) / axis) > MAX_SPEED) {motorSpeeds[axis - 1] = MAX_SPEED;}
    else if(motorSpeeds[axis - 1] + adjustmentSpeeds[1] * (abs(axis) / axis) < 0) {motorSpeeds[axis - 1] = 0;}
    else {motorSpeeds[axis - 1] += adjustmentSpeeds[1] * (abs(axis) / axis);}
      
    if(motorSpeeds[axis] + adjustmentSpeeds[1] * (abs(axis) / axis) > MAX_SPEED) {motorSpeeds[axis] = MAX_SPEED;}
    else if(motorSpeeds[axis] + adjustmentSpeeds[1] * (abs(axis) / axis) < 0) {motorSpeeds[axis] = 0;}
    else {motorSpeeds[axis] += adjustmentSpeeds[1] * (abs(axis) / axis);} 
    
  }
  
}

void MotorController::moveAxis(int axis, int distance) 
{

  if(!movingAxis[axis - 1])
  {

    movingAxis[axis - 1] = !movingAxis[axis - 1];
    moveAxis(axis);
    initialPosition = positions[axis - 1];

  }
  
  if(abs(positions[axis - 1] - initialPosition) >= distance) 
  {

    movingAxis[axis - 1] = !movingAxis[axis - 1];
    moveAxis(axis);
    
  }
  
}

void MotorController::takeOff()
{

  for(int i = 0; i < sizeof(motorPins) / sizeof(motorPins[0]; i++) {digitalWrite(motorPins[i], LOW);}

  for(int i = 0; i <= CRUISING_SPEED; i++)
  {

    for(int x = 0; i < sizeof(motorPins) / sizeof(motorPins[0]; i++) 
    {

      motorSpeeds[x] = i;
      delay(40);
      digitalWrite(motorPins[x], motorSpeeds[x]);     
      
    }

  }

}

void MotorController::land()
{

  for(int i = 0; i < sizeof(motorPins) / sizeof(motorPins[0]; i++) {digitalWrite(motorPins[i], CRUISING_SPEED);}

  for(int i = 0; i >= 0; i--)
  {

    for(int x = 0; i < sizeof(motorPins) / sizeof(motorPins[0]; i++)
    {

      motorSpeeds[x] = i;
      delay(40);
      digitalWrite(motorPins[x], motorSpeeds[x]);     
      
    }

  }

}

void MotorController::stopAllMovement()
{
  
  writeAllMotors(CRUISING_SPEED);
  for(int i = 0; i < 3; i++) {moving[i] = false;}
  
}

MotorController::~MotorController() {delete[] motorPins; delete[] moving; delete sensorMoving; delete[] motorSpeeds; delete adjustSpeeds; delete MAX_SPEED; delete CRUISING_SPEED;}
