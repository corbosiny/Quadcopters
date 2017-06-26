#ifndef PID_h
#define PID_h

#include "Arduino.h"
#include "MotorController.h"
#include "IMU.h"
#include "oAvoider.h"

class PIDcontroller
{

  public:
  static const int NUM_MOTORS = 4;     //number of motors on the drone, IDEAL NUMBER IS 4
  static const int NUM_AXIS = 4;       //number of axis to correct on, see one of the first couple pages of the flight systems manual to see axis key
  
  PIDcontroller(MotorController *motorController, IMU *imu, int constants[3],  int maxOutputs[3]);                                      //takes in a motor controller, IMU, and maxSpeed essentially
  PIDcontroller(MotorController *motorController, IMU *imu, int constants[3],  int maxOutputs[3], ObstacleAvoider *newAvoider);         //obstacle avoidance is optional for the PID controller
  
  int calcError(int axis);                                                                                                            //returns how far we are from our desired state on an axis
  void adjustAxis(int axisNum);                                                                                                       //adjusts PID outputs to get us closer to a desired state on an axis
  void changeTargets(int newTargets[NUM_MOTORS]);                                                                                     //changes all the given desired axis targets
  void changeTarget(int index, int newTarget);                                                                                        //changes the desired state target on one axis
  int *calibrateMotors();                                                                                                             //finds the motor offsets
  void newAvoider(ObstacleAvoider *newA);                                                                                             //assigns a new obstaacle avoider
  
  private:
  int maxOutputs[3];                                                                                                                  //max outputs for each PID term
  float targets[NUM_AXIS];                                                                                                            //desired state targets for each axis
  float vibrationThresh;                                                                                                              //used when calibrating the motors, helps determine when a motor has turned on based on vibrations
  float lastErrors[NUM_AXIS];                                                                                                         //used to calculate the derivative term
  boolean targetsChanged[NUM_AXIS];                                                                                                   //flags used to avoid error spikes when we change the desired state
  float integrals[NUM_AXIS];                                                                                                          //holds running errom sum on each axis
  long long int dt;                                                                                                                   //change in time since the last measurement
  float proConstant;                                                                                                                  //proportional
  float intConstant;                                                                                                                  //integral
  float derConstant;                                                                                                                  //derivative
  float *data;                                                                                                                          //the IMU system data
  IMU *imu;                                                                                                                            
  MotorController *motors;                                                                                                            
  ObstacleAvoider *avoider;                                                                                                           
  
};


#endif
