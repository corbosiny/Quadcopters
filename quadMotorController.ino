//----------------------------------------------\\
//-********************************************-\\
//-***     Feromone Motor Controller        ***-\\
//-***  Controlls and tracks motorspeeds    ***-\\
//-***         Subteam: Corey               ***-\\
//-********************************************-\\
//----------------------------------------------\\

#include <Servo.h>

int motorPins[] = {};                                                                                     //Signal pin for each of the four motors                                                            
int motorSpeedOffsets[] = {0,0,0,0};                                                                      //respective offsets for each motor on the drone, this is to help them run at the same speed
int currentMotorSpeeds[] = {0, 0, 0, 0};                                                                  //keeps track of the current motors speeds, you will see why we use this later

Servo motors[4];                                                                                          //The ESCs take signals similar to servos so we can make use of the arduino library for them
void setup() 
{

  for(int i = 0; i < 4; i++) {motors[i].attach(motorPins[i]);}                                            //attaches each pin to its respective motor
  
}

void loop() 
{

}

void writeMotor(int motorNumber, int newSpeed)                                                            //sets a new speed for an individual motor and updates the current speed variable
{
  currentMotorSpeeds[motorNumber - 1] = newSpeed + motorSpeedOffsets[motorNumber - 1];                    //adds the offset to the entered speed before writing it to the motor
  motors[motorNumber - 1].writeMicroseconds(currentMotorSpeeds[motorNumber - 1]);                         //writeMicroseconds is used instead of write as writeMicroseconds works from 1000 to 2000 instead of 0 to 180 like write, therefor more precise speed options
}

void writeMotors(int speeds[])                                                                            //takes an array of speeds and writes all motors to the respective speed assigned them
{

  for(int i = 0; i < 4; i++) 
  {
    motors[i].writeMicroseconds(speeds[i] - motorSpeedOffsets[i]);                                        //writing each motor to their respective speed and updating the speed variables while factoring in the motor offsets
    currentMotorSpeeds[i] = speeds[i] - motorSpeedOffsets[i];                                             
  }
  
}

void adjustMotors(float adjustments[])                                                                      //takes in an adjustment and will add that to the current speed of the motor, this is why we keep track of their current speeds
{
  
   for(int i = 0; i < 4; i++) {currentMotorSpeeds[i] += (int)adjustments[i];}
   writeMotors(currentMotorSpeeds);

}

void adjustMotor(int motorNumber, int adjustment) {writeMotor(motorNumber, currentMotorSpeeds[motorNumber - 1] + adjustment);} //adjust just one motor

void motorTest()                                                                                                               //just writes a bunch of values to the motors just so we can see if they are working
{

  for(int i = 0; i < 1000; i += 10) 
  {
    int t_speed = 1000 + i;
    int t_speeds[] = {t_speed, t_speed, t_speed, t_speed};
    writeMotors(t_speeds); 
    delay(100);
  }
  for(int i = 1000; i > 0; i -= 10) 
  {
    int t_speed = 1000 + i;
    int t_speeds[] = {t_speed, t_speed, t_speed, t_speed};
    writeMotors(t_speeds); 
    delay(100);
  }
  
}

void changeOffsets(int newOffsets[4]) {memcpy(motorSpeedOffsets, newOffsets, sizeof(newOffsets));}

