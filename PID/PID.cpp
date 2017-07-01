#include "PID.h"

PIDcontroller::PIDcontroller(IMU *imu, int PIDtermConstants[3], int maxPIDtermOutputs[3])
{
  this->imu = imu;
  
  memcpy(this->PIDtermConstants[3], PIDtermConstants[3], sizeof(PIDtermConstants));
  memcpy(this->maxPIDtermOutputs, maxPIDtermOutputs, sizeof(maxPIDtermOutputs));

  proportionalConstant = PIDtermConstants[0];
  integralConstant = PIDtermConstants[1];
  derivativeConstant = PIDtermConstants[2];
  resetPIDstate();
}


void PIDcontroller::resetPIDstate() 
{
   for(int axis = 0; axis < NUM_AXIS; axis++) 
   {
      integrals[axis] = 0; 
      desiredStateChanged[axis] = true;                              //used to trigger derivative error spike prevention                          
   }                 
}



float *PIDcontroller::updateStateAdjustmentsToReachDesiredStates()
{
  float adjustmentForces[NUM_AXIS];
  for(int axis = 0; axis < NUM_AXIS; axis++) 
  {
    adjustmentForces[axis] = updateStateAdjustmentToReachDesiredState(axis);
  }
  return adjustmentForces;
}

float PIDcontroller::updateStateAdjustmentToReachDesiredState(int axisNum)
{
  updateStateMeasurements(axisNum);
  float adjustmentForce = calcPIDadjustmentForce(axisNum);
  adjustmentForce = regulatePIDadjustmentForce(adjustmentForce);
  return adjustmentForce;
}

void PIDcontroller::updateStateMeasurements(int axisNum)
{
  currentStates = imu->readIMUData();
  updateStateError(axisNum);
  updateMeasurementTimer();
}

void PIDcontroller::updateStateError(int axisNum)
{
  int newStateError = calcStateError(axisNum);
  if(desiredStateChanged[axisNum]) {lastStateErrors[axisNum] = newStateError;}                                 //prevents initial spikes in derivative error when a state has changed        
  else 
  {
    lastStateErrors[axisNum] = currentStateErrors[axisNum]; 
    currentStateErrors[axisNum] = newStateError;
  }
}

void PIDcontroller::updateStateErrors()
{
  for(int axis = 0; axis < NUM_AXIS; axis++) {updateStateError(axis);}  
};


float PIDcontroller::calcStateError(int axisNum) 
{
  return (desiredStates[axisNum] - currentStates[axisNum]);
}

float *PIDcontroller::calcStateErrors()
{
  float currentStateErrors[NUM_AXIS];
  for(int axis = 0; axis < NUM_AXIS; axis++) {currentStateErrors[axis] = calcStateError(axis);}
  return currentStateErrors;
}

void PIDcontroller::updateMeasurementTimer()
{
 secondsSinceLastMeasurement = calcSecondsSinceLastMeasurement();
 timeLastMeasurementTaken = millis();  
}

int PIDcontroller::calcSecondsSinceLastMeasurement() {return (millis() - timeLastMeasurementTaken) / 1000;}

float PIDcontroller::calcPIDadjustmentForce(int axisNum)
{
  float *terms = calcPIDadjustmentForceTerms(axisNum);
  float *regulatedTerms = regulatePIDterms(terms);
  return regulatedTerms[0] + regulatedTerms[1] + regulatedTerms[2];
}

float *PIDcontroller::calcPIDadjustmentForceTerms(int axisNum)
{
  float terms[3];
  terms[0] = calcProportionalAdjustmentForce(axisNum);
  terms[1] = calcIntegralAdjustmentForce(axisNum);
  terms[2] = calcDerivativeAdjustmentForce(axisNum);
  return terms;
}

float PIDcontroller::calcProportionalAdjustmentForce(int axisNum) {return proportionalConstant * currentStateErrors[axisNum];}

float PIDcontroller::calcIntegralAdjustmentForce(int axisNum) 
{
  integrals[axisNum] += (currentStateErrors[axisNum] / secondsSinceLastMeasurement) * integralConstant; 
  return integrals[axisNum];
}

float PIDcontroller::calcDerivativeAdjustmentForce(int axisNum) 
{
  return derivativeConstant * (currentStateErrors[axisNum] - lastStateErrors[axisNum]) / secondsSinceLastMeasurement;
} 

float *PIDcontroller::regulatePIDterms(float unregulatedTerms[])
{
  float regulatedTerms[3];
  for(int termNum = 0; termNum < 3; termNum++)
  {
    regulatedTerms[termNum] = regulatePIDterm(termNum, unregulatedTerms[termNum]);
  }
  return regulatedTerms;
}

float PIDcontroller::regulatePIDterm(int termNum, float termValue)
{
  if(abs(termValue) > maxPIDtermOutputs[termNum]) {termValue = maxPIDtermOutputs[termNum] * (termValue / abs(termValue));}       //x / abs(x) gives you the sign of the number we are preserving the sign of the adjustment force
  return termValue;
}

float PIDcontroller::regulatePIDadjustmentForce(float unregulatedForce)
{
  if(abs(unregulatedForce) > MAX_OUTPUT_FORCE) {return MAX_OUTPUT_FORCE * (unregulatedForce / abs(unregulatedForce));}   //x / abs(x) gives you the sign of the number, we are preserving the sign of the adjustment force
  else{return unregulatedForce;}
}

//getters and setters
int *PIDcontroller::getPIDconstants() {return PIDtermConstants;}
void PIDcontroller::setPIDconstants(int newConstants) {memcpy(PIDtermConstants, newConstants, sizeof(PIDtermConstants));}

int *PIDcontroller::getMaxPIDtermOutputs() {return maxPIDtermOutputs;}
void PIDcontroller::setMaxPIDtermOutputs(int newMaxTermOutputs[]) {memcpy(maxPIDtermOutputs, newMaxTermOutputs, sizeof(maxPIDtermOutputs));}

float *PIDcontroller::getDesiredStates() {return desiredStates;}
void PIDcontroller::setDesiredStates(float newDesiredStates[NUM_AXIS])
{
   memcpy(desiredStates, newDesiredStates, sizeof(newDesiredStates));
   resetPIDstate();
}

void PIDcontroller::setDesiredState(int axisNum, float newTarget)
{
  desiredStates[axisNum] = newTarget;
  integrals[axisNum] = 0;
  desiredStateChanged[axisNum] = true;                                                          //used to trigger derivative error spike prevention  
}

