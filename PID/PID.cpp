#include "PID.h"

PIDcontroller::PIDcontroller(MotorController *motorController, IMU *imu, int constants[3], int maxOutputs[3])
{
  this->imu = imu;
  this->motors = motorController;
  memcpy(this->maxOutputs, maxOutputs, sizeof(maxOutputs));

  proConstant = constants[0];
  intConstant = constants[1];
  derConstant = constants[2];
  for(int i = 0; i < NUM_AXIS; i++) {targetsChanged[i] = true; integrals[i] = 0;}
}

PIDcontroller::PIDcontroller(MotorController *motorController, IMU *imu, int constants[3], int maxOutputs[3], ObstacleAvoider *newAvoider)
{
  this->imu = imu; 
  motors = motorController;
  memcpy(this->maxOutputs, maxOutputs, sizeof(maxOutputs));

  proConstant = constants[0];
  intConstant = constants[1];
  derConstant = constants[2];
  this->avoider = newAvoider;
  for(int i = 0; i < NUM_AXIS; i++) {targetsChanged[i] = true; integrals[i] = 0;}
}

void PIDcontroller::newAvoider(ObstacleAvoider *newA) {avoider = newA;}

int PIDcontroller::calcError(int axis) {return (data[axis] - targets[axis]);}

void PIDcontroller::adjustAxis(int axisNum)
{

  data = imu->readIMUData();
  float error = data[axisNum] - targets[axisNum];                                     //calculates error term then uses the PID formulas to get the Proportional, integral, and derivative term
  
  float proportional = proConstant * error;
  if(abs(proportional) > maxOutputs[0]) {proportional = maxOutputs[0] * (error / abs(error));}
  
  if(targetsChanged[axisNum] == true) {targetsChanged[axisNum] = false; lastErrors[axisNum] = error;}       //resetting the flags for changed targets and setting the last errors to the current error to essentially turn the derivative to zero to avoid over compensation                                        
  
  float derivative = derConstant * (error - lastErrors[axisNum]) / (millis() - dt);
  if(abs(derivative) > maxOutputs[2]) {derivative = maxOutputs[2] * (derivative / abs(derivative));}
  
  integrals[axisNum] += (error / (millis() - dt)) * intConstant;
  if(abs(integrals[axisNum]) > maxOutputs[1]) {integrals[axisNum] = maxOutputs[1] * (integrals[axisNum] / abs(integrals[axisNum]));}

  lastErrors[axisNum] = error;                                                          //updates the last error term that the derivative uses                          

  float output = 0;
  //We divide by two to split the adjustment between the two sets of motors, one set's speed is always increased, while ones is always decreased. splits the burden
  if(avoider != NULL && axisNum == 1) {output = (proportional + derivative + integrals[axisNum] - avoider->obstacleAvoidAxis(0, maxOutputs[0], maxOutputs[1], maxOutputs[2]) + avoider->obstacleAvoidAxis(2, maxOutputs[0], maxOutputs[1], maxOutputs[2])) / 2;}
  else if(avoider != NULL && axisNum == 2) {output = (proportional + derivative + integrals[axisNum] - avoider->obstacleAvoidAxis(1, maxOutputs[0], maxOutputs[1], maxOutputs[2]) + avoider->obstacleAvoidAxis(3, maxOutputs[0], maxOutputs[1], maxOutputs[2])) / 2;}                 
  else {(proportional + derivative + integrals[axisNum]) / 2;}
  
  if(abs(output) > maxOutputs[0] + maxOutputs[1] - maxOutputs[2]) {output = (maxOutputs[0] + maxOutputs[1] + maxOutputs[2]) * (output / abs(output));}
  
  if(axisNum == 4) {output *= 2;}                                                       //if we are adjusting altitude, we undo the one half as all the motors will go one way
  float outputs[NUM_MOTORS];
  for(int i = 0; i < NUM_MOTORS; i++) {outputs[i] = output;}                            //we put it in an arry format to work with the writeMotors command from the motorController library                              
  switch(axisNum)                                                                       //different axis divide the motors into different sets, this switch figures out which we are working on
  {
    case 0:                                                                             //pitch, split into front two and back two, the back two get the opposite adjust from the front two
    for(int i = NUM_MOTORS / 2; i < NUM_MOTORS; i++) {outputs[i] *= -1;}
    motors->adjustMotors(outputs);  
    break;
    
    case 1:                                                                             //roll, split into left two and right two motors, right two always get the opposite adjust from the right two 
    outputs[NUM_MOTORS / 2 - 1] *= -1;
    outputs[NUM_MOTORS / 2] *= -1;
    motors->adjustMotors(outputs);
    break;
    
    case 2:                                                                             //yaw, splits the motors along the left and right diagnols, one goes clockwise while the other spinds counter, the right diagnol always gets the opposite offset of he left diagnol
    outputs[0] *= -1;                                                                   //This line assumes that motors 1 and 3 are the clockwise motors, just flip the signs of everyone if it is the opposite
    outputs[NUM_MOTORS - 1] *= -1;
    motors->adjustMotors(outputs);
    break;
    
    case 3:                                                                             //altitude, all motors get the same adjust as the entire drone goes up or down to adjust this
    motors->adjustMotors(outputs);
    break;
    
  }
  
}

void PIDcontroller::changeTargets(int newTargets[NUM_AXIS])
{

   memcpy(targets, newTargets, sizeof(newTargets));
   for(int i = 0; i < NUM_AXIS; i++) 
   {
      integrals[i] = 0; 
      targetsChanged[i] = true;                                                          //setting all the flags in targetsChanged to signal a reset of the derivative and integral terms to avoid overcompensation
   }                 
  
}

void PIDcontroller::changeTarget(int index, int newTarget)
{
  targets[index] = newTarget;
  integrals[index] = 0;
  targetsChanged[index] = true;                                                             //setting the flag in targetsChanged to signal a reset of the derivative and integral terms to avoid overcompensation
}

int *PIDcontroller::calibrateMotors()
{                                                                          
   imu->readMPUDataRaw();                                                                    //updating
   float mag = sqrt(sq(imu->accX) + sq(imu->accY));                                           //magnitude of the vibrations
   int newOffsets[NUM_MOTORS];                                                           
   int turnOnSpeeds[NUM_MOTORS];                                                            //will hold the voltages the motors turn on at
   int currentSpeed = 0;                                                                    //holds the current speed we are testing
   for(int i = 0; i < NUM_MOTORS; i++)                                                               //will run through every motor
   {
      newOffsets[i] = 0;
      turnOnSpeeds[i] = 0;
      while(mag < vibrationThresh)                                                          //as long as the magnitude of the vibrations is small enough, we assume the motor isn't fully turned on
      {

        currentSpeed += 10;                                                                 
        motors->writeMotor(i, currentSpeed);                                                 //write motors more and more speed
        imu->readMPUDataRaw();                                                               //read the MPU data
        mag = sqrt(sq(imu->accX) + sq(imu->accY));                                            //calculating magnitude of vibrations
        
      }

       turnOnSpeeds[i] = currentSpeed;                                                      //store speeds
       
   }
   
   int topSpeed = turnOnSpeeds[0];
   for(int i = 0; i < NUM_MOTORS; i++) {if(turnOnSpeeds[i] > topSpeed) {topSpeed = turnOnSpeeds[i];}}     //take max start on speed, calculate offsets to make them all that speed
                                                 
   for(int i = 0; i < NUM_MOTORS; i++) {newOffsets[i] = turnOnSpeeds[i] - topSpeed;}        //calculating offsets
   motors->changeOffsets(newOffsets);                                                        //setting current offsets to the new ones
   return newOffsets;
}

