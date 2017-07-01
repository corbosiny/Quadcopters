//             @
//            @@@
//         @@@@@@@@@
//    @@@@@@@@@@@@@@@@@@@@
//  @@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// ##########################
// $  Feromone PID Control  $
// $   keeps drone steady   $
// $  Subteam: Corey, Ali   $
// ########################## 
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//PID stands for proportional integral derivative controller
//it essentially will be given a desired state(certain pitch, roll, and yaw rotations) and adjust to that state
//there are some links to videos on the math behind it on the drive, a quick youtube search will get you some results too

#include "MotorController.h"
#include "IMU.h"
#include "oAvoider.h"

int pins[4] = {2,3,4,5};
IMU imu(30.2);                                      //insert actual sea level pressure here                                 

int targets[4] = {1000, 1000, 1000, 1000};          //pitch, roll, yaw, altitude
int lastErrors[4] = {0,0,0,0};                      //used for derivative calculations, order: pitch, roll, yaw, altitude
int targetsChanged[4] = {false, false, false, false}; //used to reset integral and derivative terms when targets are changed to avoid over compensation
int integrals[4] = {0,0,0,0};                       //holds the integral components, order: pitch, roll, yaw, altitude
long long int dt = millis();                        //holds the time since the last calculation, used in all time based calculations

int proConstant = 1;                                //constants for each PID term
int intConstant = 1;                                
int derConstant = 1;

void setup() 
{

    Serial.begin(9600);

}

void loop() 
{

  

}


