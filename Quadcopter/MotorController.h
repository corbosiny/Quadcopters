#ifndef MotorController_h
#define MotorController_h

#include "Arduino.h"

class MotorController
{

  friend class Quad; //allows Quad to access private variables
  
  public:
  MotorController(int p_motorPins[]);
  
  void takeOff();
  void land();
  
  void writeAllMotors(bool state);
  void writeAllMotors(int pwmSignal);
  void writeAllMotors(int speeds[]);

  void moveAxis(int axis, int undoing); //moves indefinetely down one axis until this function is called again, negative numbers make it move in the opposite direction
  void moveAxisSensor(int axis, int undoing);
  void moveAxis(int axis, int distance); //moves the Quad copter a certain distance down one axis, negtaive numbers work here

  void writeMotor(int motorNum, bool state);
  void writeMotor(int motorNum, int pwmSignal);

  void stopAllMovement(); //can you guess what this does?

  ~MotorController();

  private:
  int motorPins[4]; //first pin is motor one, second, is motor 2, third is motor 3, fourth is motor 4

  bool moving[3] = {false, false, false}; //tells whether the quad copter is moving down a certain address
  bool sensorMoving[3] = {false, false, false};
  
  int motorSpeeds[4] = {0,0,0,0}; //holds the current motor speeds so we can easily adjust them for turns and such
  
  int adjustSpeeds[2] = {40, 60}; //first index is turning adjust speed, second index is obstacle avoidance adjust speed

  int MAX_SPEED = 185; //max speed the quad copter will go, do not change unless you know what you are doing
  int CRUISING_SPEED = 120; //normal hovering speed

  
};


#endif
