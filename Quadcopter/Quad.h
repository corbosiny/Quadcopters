#ifndef Quad_h
#define Quad_h 

#include "Arduino.h"
#include "PositionTracker.h"
#include "ObstacleAvoider.h"
#include "MotorController.h"
#include "Balancer.h"

//  ****************************
//  *** Repeater Quad Copter ***
//  **** Pheromone Robotics ****
//  ****************************

/**************************************************************************************
$~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~$        
* as a general rule, any function that requires an integer input for an axis          *
*uses 1 for the X axis, 2 for the Y axis, and 3 for the Z axis                        *
*also motors 1 and 2 face the positive side of the y axis, and the numbering          *
*goes clockwise from the top left motor, which starts at one                          *
*the accelerometer is read from the onboard ones on each motor                        *
*we hacked on to arduino pins and the arduino makes adjustments based on              *
*the readings readings coming from them, the robot will line up, within a             *
*given tolerance, to user position readings from a wearable transmitter               *
*also the robot needs another transmitter to tell it where the battery pad is         *
*to charge at, so pressing the signal button on the charge pad will allow the         *
*quad copter to know where it is at, also there are plans for the future where        *
*the pads board will let the quad copter know what pads are available for landing     *
*and possibly manage recruitment calls by deciding who has had the most time charging *
*Note: calling negative numbers in the move functions causes movement in opposite     *                                                                            
*directions, and you must recall the negative number to undo this                     *
$~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~$
***************************************************************************************/

class Quad
{

  public:
  Quad(int p_motorPins[], int p_accelerometerPins[], int p_sensorPins[], p_ESPPins[]); //tolerances are predefined, but manual ones can be entered into the overloaded constructor
  Quad(int p_motorPins[], int p_accelerometerPins[], int p_sensorPins[], p_ESPPins[], int p_tolerances[]); 

  void setMotorPin(int motorNum, int newPin);
  int getMotorPin(int motorNum);
  
  int readMotor(int motorNum);

  void setAccelerometerPin(int numAccel, int newPin);
  int getAccelerometerPin(int numAccel);
  
  void setESPPin(int numPim, int newPin); //used for wifi, one is tx and two is rx
  int getESPPin(int numPin);

  void setNetworkName(String newName); String getNetworkName(); void setNetworkPassword(String newPassword); String getNetworkPassword(); void setAdjustSpeed(int adjustSpeed, int newSpeed); //how fast the quadcopter avoids obstacles
  int getAdjustSpeed(int adjustSpeed);

  void setTolerance(int axis, int newTolerance); //the tolerance the quadcopter will align on an axis with, higher tolerance means higher presicion allignment
  void setAllTolerances(int newTolerance);
  int getTolerance(int axis);

  void setMaxSpeed(int newMaxSpeed); 
  int getMaxSpeed();

  void setCruisingSpeed(int newCruisingSpeed); //hover speed
  int getCruisingSpeed();

  void setPosition(int axis, int newPosition); //allows the user to recalibrate the quad copters relative position, NOT RECOMMENDED TO USE EVER, I MEAN IT, DON'T CALL THIS MILES WHILE TESTING!
  void setAllPositions(int p_position); //allows the user to recalibrate the quad copters relative position, NOT RECOMMENDED TO USE EVER, I MEAN IT, DON'T CALL THIS MILES WHILE TESTING!
  void setAllPositions(int p_positions[]); //allows the user to recalibrate the quad copters relative position, NOT RECOMMENDED TO USE EVER, I MEAN IT, DON'T CALL THIS MILES WHILE TESTING!
  int getPosition(int axis); //returns its relative position on any axis

  void setSensorPin(int sensorNum, int newPin);
  int getSensorPin(int sensorNum);

  void setDistanceThreshold(int newThresh); //sets how close an object can get until the quadcopter avoids it
  int getDistanceThreshold();

  void setBatteryPin(int newPin);
  int getBatteryPin();
  float readBattery(); //reads battery voltage, used to trigger a return home call if battery is low

  void setBatteryThreshold(); //Determines when the quadcopter returns to charge
  void getBatteryThreshold(); 
  
  void manualMode(); //allows user to directly control the quadcopter

  ~Quad(); //Deconstructor
   
  private:
  float BATTERY_THRESHOLD = 1.5; //Determines when the quadcopter returns to charge, units are volts
  int batteryPin = A0;
  
  int ESPPins[2];
  
  
};

#endif
