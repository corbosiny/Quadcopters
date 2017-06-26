#ifndef MotorController_h
#define MotorController_h

#include "Arduino.h"
#include <Servo.h>

class MotorController
{

  friend class PID;

  public:
    static const int NUM_MOTORS = 4;
    MotorController(int motorPins[NUM_MOTORS], int motorSpeedOffsets[NUM_MOTORS] = {});

    void writeMotor(int motorNumber, int newSpeed);                                         //sets the speed of one motor
    void writeMotors(int speeds[]);                                                         //sets the speed of all motors
    void adjustMotors(float adjustments[]);                                                 //adjust the speed of all motors
    void adjustMotor(int motorNumber, int adjustment);                                      //adjust the speed of one motor
    void motorTest();                                                                       //just runs the motors through several speeds to make sure they are working
    void changeOffsets(int newOffsets[NUM_MOTORS]);

  private:                                                                                 
    int motorPins[NUM_MOTORS];                                                                      //holds the signal pin for each motors ESC
    Servo motors[NUM_MOTORS];                                                                       //each ESC take signals like that of a servo, so we can use the Servo class to handle all the specifics of generating signals
    int motorSpeedOffsets[NUM_MOTORS];                                                              //holds the offsets we use to make the motors run at the same speed
    int currentMotorSpeeds[NUM_MOTORS];                                                             //keeps track of the speed of every motor
  
};

#endif
