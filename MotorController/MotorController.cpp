#include "MotorController.h"

MotorController::MotorController(int p_motorPins[4], int p_motorSpeedOffsets[4] = {})                 //takes in the motorpins and offsets
{

  memcpy(motorPins, p_motorPins, sizeof(p_motorPins));
  memcpy(motorSpeedOffsets, p_motorSpeedOffsets, sizeof(p_motorSpeedOffsets));

  for(int i = 0; i < NUM_MOTORS; i++) {motors[i].attach(motorPins[i]); currentMotorSpeeds[i] = 0;}                           //attachs each of the motor's servo object to their respective pin
  
}

void MotorController::writeMotor(int motorNumber, int newSpeed)                                       //Sets speed of one motor                                       
{
  currentMotorSpeeds[motorNumber - 1] = newSpeed - motorSpeedOffsets[motorNumber - 1];                //Edits stores speed and factors in offset before writing to the motor
  motors[motorNumber - 1].writeMicroseconds(currentMotorSpeeds[motorNumber - 1]);                     
}

void MotorController::writeMotors(int speeds[])                                                       //Takes in array of speeds and writes it to all motors
{

  for(int i = 0; i < NUM_MOTORS; i++)                                                        
  {
    motors[i].writeMicroseconds(speeds[i] - motorSpeedOffsets[i]);                                    //Updates tracked speeds and factors in offsets
    currentMotorSpeeds[i] = speeds[i] - motorSpeedOffsets[i];
  }
  
}

void MotorController::adjustMotors(float adjustments[])                                                    //Adjust speeds of all the motors in relation to their current speeds, this is why we keep track of their current speeds 
{

   for(int i = 0; i < NUM_MOTORS; i++) {currentMotorSpeeds[i] += (int)adjustments[i];}                     //adds adjustments to each motors speed
   writeMotors(currentMotorSpeeds);

}

void MotorController::adjustMotor(int motorNumber, int adjustment) {writeMotor(motorNumber, currentMotorSpeeds[motorNumber - 1] + adjustment);} //adjust one motors speed, makes use of the write motor function to do it

void MotorController::motorTest()                                                                                                               //Just writes the motors through a bunch of values for us to see if they work smoothly or not
{

  for(int i = 0; i < 1000; i += 10) {int testSpeeds[4] = {1000 + i, 1000 + i, 1000 + i, 1000 + i}; writeMotors(testSpeeds); delay(100);}
  for(int i = 1000; i > 0; i -= 10) {int testSpeeds[4] = {1000 + i, 1000 + i, 1000 + i, 1000 + i}; writeMotors(testSpeeds); delay(100);}
  
}

void MotorController::changeOffsets(int newOffsets[NUM_MOTORS]) {memcpy(motorSpeedOffsets, newOffsets, sizeof(newOffsets));}
