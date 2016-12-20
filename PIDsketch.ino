             @
            @@@
         @@@@@@@@@
    @@@@@@@@@@@@@@@@@@@@
  @@@@@@@@@@@@@@@@@@@@@@@@
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
 ##########################
 $  Feromone PID Control  $
 $   keeps drone steady   $
 $  Subteam: Corey, Ali   $
 ########################## 
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//PID stands for proportional integral derivative controller
//it essentially will be given a desired state(certain pitch, roll, and yaw rotations) and adjust to that state
//there are some links to videos on the math behind it on the drive, a quick youtube search will get you some results too

#include <MotorController.h>
#include <IMU.h>

int pins[4] = {2,3,4,5};
MotorController motors(pins);
IMU imu(30);

int targetRoll;
int targetPitch;
int targetAltitiude;

void setup() 
{

    

}

void loop() 
{


}

void adjustState()                                                          //goes through every aspect of the drones rotations and adjust them toward the desired state
{

  imu.readIMUdata()
  adjustAltitude();
  adjutPitch();
  adjustRoll();
  adjustYaw();
  
}

void adjustAltitiude()                                                      //adjust altitude toward desired altitude by either raising or lowering group motor speed
{


}

void adjustPitch()                                                          //adjust pitch toward desired pitch by adjusting speed of front and back motors
{


}

void adjustRoll()                                                           //adjust roll toward desired roll by adjusting speed of left and right motors
{


}

void adjustYaw()                                                            //uses magenometer to keep the drone pointed to a certain bearing in relation to north(which we have as 0/360 degreees)
{


}

