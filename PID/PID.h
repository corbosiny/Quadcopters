#ifndef PID_h
#define PID_h

#include "Arduino.h"
#include "oAvoider.h"
#include "IMU.h"

class PIDcontroller
{

  public:   
  static const int NUM_AXIS = 4; 
  static const int MAX_OUTPUT_FORCE = 500;
    
  PIDcontroller(IMU *imu, int PIDtermConstants[3], int maxPIDtermOutputs[3]);
  PIDcontroller(IMU *imu, int PIDtermConstants[3], int maxPIDtermOutputs[3], ObstacleAvoider *newAvoider);                                           
  
  void resetPIDstate();
  float *updateStateAdjustmentsToReachDesiredStates();
  float updateStateAdjustmentToReachDesiredState(int axisNum);

  void updateStateMeasurements(int axisNum);
  void updateStateError(int axisNum);
  void updateStateErrors();
  float calcStateError(int axisNum);
  float *calcStateErrors();
  
  void updateMeasurementTimer();
  int calcSecondsSinceLastMeasurement();
  
  float calcDesiredStateAdjustmentForce(int axisNum);
  float calcPIDadjustmentForce(int axisNum);
  float *calcPIDadjustmentForceTerms(int axisNum);
  float calcProportionalAdjustmentForce(int axisNum);
  float calcIntegralAdjustmentForce(int axisNum);
  float calcDerivativeAdjustmentForce(int axisNum);
  float *regulatePIDterms(float unregulatedTerms[]);
  float regulatePIDterm(int termNum, float termValue);
  float getObstacleAvoidanceForces(int axisNum);
  float regulateDesiredStateAdjustmentForce(float unregulatedForce);

  void newAvoider(ObstacleAvoider *newAvoider);                                                                                             

  int *getPIDconstants();
  void setPIDconstants(int newConstants);
  int *getMaxPIDtermOutputs();
  void setMaxPIDtermOutputs(int newMaxTermOutputs[]);
  float *getDesiredStates();
  void setDesiredStates(float newDesiredStates[NUM_AXIS]);
  void setDesiredState(int axisNum, float newTarget); 
  
  private:
  int PIDtermConstants[3];                      //0 = proportional, 1 = integral, 2 = derivative
  int maxPIDtermOutputs[3];                                                                                                                                                  
  float proportionalConstant;                                                                                                                  
  float integralConstant;                                                                                                                 
  float derivativeConstant;   
  
  float desiredStates[NUM_AXIS];                                                                                                                                                                                
  float lastStateErrors[NUM_AXIS];
  float currentStateErrors[NUM_AXIS];                                                                                                         
  boolean desiredStateChanged[NUM_AXIS];                                                                                                   
  float integrals[NUM_AXIS];                                                                                                          
  long long int timeLastMeasurementTaken;
  float secondsSinceLastMeasurement;                                                                                                                   
                                                                                                             
  float *currentStates;                                                                                                                          
  IMU *imu;                                                                                                                                                                                                                                      
  ObstacleAvoider *obstacleAvoider;                                                                                                           
  
};


#endif
