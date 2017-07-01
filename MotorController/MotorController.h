#ifndef MotorController_h
#define MotorController_h

#include "Arduino.h"
#include <Servo.h>

class MotorController
{

  friend class PID;

  public:
    static const int NUM_MOTORS = 4;
    static int MAX_MOTOR_SPEED;
    static int MIN_MOTOR_SPEED;
    
    MotorController(int motorPins[NUM_MOTORS], int motorSpeedOffsets[NUM_MOTORS] = {});

    void updateMotorSpeeds(int newMotorSpeeds[NUM_MOTORS]);
    void updateMotorSpeed(int motorNum, int newMotorSpeed);
    void incrementAndWriteMotorSpeeds(float increments[NUM_MOTORS]);                                                 
    void incrementAndWriteMotorSpeed(int motorNumber, int increment = 1);

    int *offsetMotorSpeeds(int motorSpeeds[NUM_MOTORS]);
    int offsetMotorSpeed(int motorNum, int motorSpeed);
    int *limitMotorSpeeds(int motorSpeeds[NUM_MOTORS]);
    int limitMotorSpeed(int motorSpeed);

    void writeMotorsNewSpeeds(int newMotorSpeeds[NUM_MOTORS]); 
    void writeMotorNewSpeed(int motorNumber, int newSpeed);                                                                                                                                      
    
    void motorTest();                                                                       
    
    int *getMotorSpeedOffsets();
    void setMotorSpeedOffsets(int newOffsets[NUM_MOTORS]);
    int *getMotorPins();
    void setMotorPins(int newMotorPins[NUM_MOTORS]);
    
  private:                                                                                 
    int motorPins[NUM_MOTORS];                                                                      
    Servo motors[NUM_MOTORS];                                                                       //each ESC take signals like that of a servo, so we can use the prebuilt class to work with them
    int motorSpeedOffsets[NUM_MOTORS];                                                              
    int currentMotorSpeeds[NUM_MOTORS];                                                             
  
};

#endif
