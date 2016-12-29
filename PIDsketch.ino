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

#include <MotorController.h>
#include <IMU.h>

int pins[4] = {2,3,4,5};
MotorController motors(pins);                       //change pins to whatever you need 
IMU imu(30.2);                                      //insert actual sea level pressure here                                 

int targets[4] = {1000, 1000, 1000, 1000};          //pitch, roll, yaw, altitude
int lastErrors[4] = {0,0,0,0};                      //used for derivative calculations, order: pitch, roll, yaw, altitude
int targetsChanged[4] = {false, false, false, false}; //used to reset integral and derivative terms when targets are changed to avoid over compensation
int integrals[4] = {0,0,0,0};                       //holds the integral components, order: pitch, roll, yaw, altitude
long long int dt = millis();                        //holds the time since the last calculation, used in all time based calculations

int proConstant = 1;                                //constants for each PID term
int intConstant = 1;                                
int derConstant = 1;


int testStateOverAxis = 1;                           //use for one to test the adjust state function and zero to test the adjustAxis function 
void setup() 
{

    Serial.begin(9600);

}

void loop() 
{

    if(testStateOverAxis == 1)
    {
        adjustState();                                  //adjust everything
        delay(200);
    }
    else if(!testStateOverAxis) {
        adjustAxis(0);                                  //pitch
        adjustAxis(1);                                  //roll
        adjustAxis(2);                                  //yaw
        adjustAxis(3);                                  //altitude
        delay(200);
    }
    else {
        
        int *newOffsets = calibrateMotors();
        for(int i = 0; i < 4; i++) {Serial.print("Offset: "); Serial.println(newOffsets[i]);}
    }

}

void adjustState()                                                          //goes through every aspect of the drones rotations and adjust them toward the desired state
{

  imu.readIMUdata();
  adjustAltitude();
  adjustPitch();
  adjustRoll();
  adjustYaw();
  dt = millis();                                                          //reset DT so we know the time interval between calculations
  
}


//***Each of the "adjustXXX functions have identical beginnings so I only fully commented the beginnings of adjustPitch and adjustAxis***
void adjustPitch()                                                          //adjust pitch toward desired pitch by adjusting speed of front and back motors
{

  int error = imu.anglePitch - targets[0];                                  //calculates error term
  int proportional = proConstant * error;                                   //calculates all the parts of the PID term that use the error(Proportional, Integral, Derivative)
  int derivative = derConstant * (error - lastErrors[0]) / dt;
  integrals[0] += (error / (millis() - dt)) * intConstant;

  lastErrors[1] = error;                                                    //updates last error for the next derivative calcualtion
  int output = (proportional + derivative + integrals[0]) / 2;              //makes our output amount and divide by two as each set of motors will take half of the adjust for the adjustMotors function of the motorController library, this will only adjust the currentMotorspeeds according to our PID outputs

  int outputs[4] = {output, output, -output, -output};                      //we flip the signs of the back two motors as they should be adjusted in the opposite direction of the front motors, this will always ensure that
  motors.adjustMotors(outputs);
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("Proportional: ");
  Serial.println(proportional);
  Serial.print("Derivative: ");
  Serial.println(derivative);
  Serial.print("Integral: ");
  Serial.println(integrals[0]);
  Serial.print("Output: ");
  Serial.println(output);
  Serial.println("\n\n");
}

void adjustRoll()                                                           //adjust roll toward desired roll by adjusting speed of left and right motors
{

  int error = imu.angleRoll - targets[1];
  int proportional = proConstant * error;
  int derivative = derConstant * (error - lastErrors[1]) / (millis() - dt);
  integrals[1] += (error / (millis() - dt)) * intConstant;

  lastErrors[1] = error;
  int output = (proportional + derivative + integrals[1]) / 2;

  int outputs[4] = {output, -output, -output, output};              //we flip the right motors as they should always be adjusted in the opposite direction as the left motors
  motors.adjustMotors(outputs);
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("Proportional: ");
  Serial.println(proportional);
  Serial.print("Derivative: ");
  Serial.println(derivative);
  Serial.print("Integral: ");
  Serial.println(integrals[1]);
  Serial.print("Output: ");
  Serial.println(output);
  Serial.println("\n\n");
}

void adjustYaw()                                                            //uses magenometer to keep the drone pointed to a certain bearing in relation to north(which we have as 0/360 degreees)
{

  int error = imu.heading - targets[2];
  int proportional = proConstant * error;
  int derivative = derConstant * (error - lastErrors[2]) / (millis() - dt);
  integrals[2] += (error / (millis() - dt)) * intConstant;

  lastErrors[2] = error;
  int output = (proportional + derivative + integrals[2]) / 2;

  
  int outputs[4] = {output, -output, output, -output};              //This code assumes that motors 1 and 3 are the clockwise motors, just flip the signs of everyone if it is the opposite
  motors.adjustMotors(outputs);                                     //we flip the signs of the right diagnol as it should always be adjusted in the opposite direction of the left diagnol
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("Proportional: ");
  Serial.println(proportional);
  Serial.print("Derivative: ");
  Serial.println(derivative);
  Serial.print("Integral: ");
  Serial.println(integrals[3]);
  Serial.print("Output: ");
  Serial.println(output);
  Serial.println("\n\n");
}

void adjustAltitude()                                                      //adjust altitude toward desired altitude by either raising or lowering group motor speed
{

  int error = imu.altitude - targets[3];
  int proportional = proConstant * error;
  int derivative = derConstant * (error - lastErrors[3]) / (millis() - dt);
  integrals[3] += (error / (millis() - dt)) * intConstant;

  lastErrors[3] = error;
  int adjust = proportional + integrals[3] + derivative;

  int adjusts[4] = {adjust, adjust, adjust, adjust};                      //to adjust altitude all the motors must be adjusted in the same way, so we flip no signs
  motors.adjustMotors(adjusts); 
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("Proportional: ");
  Serial.println(proportional);
  Serial.print("Derivative: ");
  Serial.println(derivative);
  Serial.print("Integral: ");
  Serial.println(integrals[3]);
  Serial.print("Output: ");
  Serial.println(adjust);
  Serial.println("\n\n");
}

void adjustAxis(int axisNum)                                                            //0 = pitch, 1 = roll, 2 = yaw, 3 = altitude
{

  int *data = imu.readIMUdata();
  int error = data[axisNum] - targets[axisNum];                                     //calculates error term then uses the PID formulas to get the Proportional, integral, and derivative term
  int proportional = proConstant * error;                                               
  int derivative = derConstant * (error - lastErrors[axisNum]) / (millis() - dt);
  integrals[axisNum] += (error / (millis() - dt)) * intConstant;

  lastErrors[axisNum] = error;                                                          //updates the last error term that the derivative uses                          

  int output = (proportional + derivative + integrals[axisNum]) / 2;                    //We divide by two to split the adjustment between the two sets of motors, one set's speed is always increased, while ones is always decreased. splits the burden
  if(axisNum == 4) {output *= 2;}                                                       //if we are adjusting altitude, we undo the one half as all the motors will go one way
  int outputs[4] = {output, output, output, output};                                    //we put it in an arry format to work with the writeMotors command from the motorController library
  
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("Proportional: ");
  Serial.println(proportional);
  Serial.print("Derivative: ");
  Serial.println(derivative);
  Serial.print("Integral: ");
  Serial.println(integrals[0]);
  Serial.print("Output: ")
  Serial.println(output);
  Serial.println("\n\n");
   
  switch(axisNum)                                                                       //different axis divide the motors into different sets, this switch figures out which we are working on
  {
    case 0:                                                                             //pitch, split into front two and back two, the back two get the opposite adjust from the front two
    outputs[2] *= -1;
    outputs[3] *= -1; 
    break;
    
    case 1:                                                                             //roll, split into left two and right two motors, right two always get the opposite adjust from the right two 
    outputs[1] *= -1;
    outputs[2] *= -1;
    break;
    
    case 2:                                             //yaw, splits the motors along the left and right diagnols, one goes clockwise while the other spinds counter, the right diagnol always gets the opposite offset of he left diagnol
    outputs[1] *= -1;                                   //This line assumes that motors 1 and 3 are the clockwise motors, just flip the signs of everyone if it is the opposite
    outputs[3] *= -1;
    break;
    
    case 3:                                             //altitude, all motors get the same adjust as the entire drone goes up or down to adjust this
    break;
    
  }
  
    motors.adjustMotors(outputs);
    
}

void changeTargets(int newTargets[4])                                                        //adjust target state values, used so that a sudden change in targets won't look like a massive movement from the target to the integral and especially derivative component
{

   memcpy(targets, newTargets, sizeof(newTargets));
   lastErrors[0] = targets[0] - imu.anglePitch;
   lastErrors[1] = targets[1] - imu.angleRoll;
   lastErrors[2] = targets[2] - imu.heading;
   lastErrors[3] = targets[3] - imu.altitude;
   for(int i = 0; i < 4; i++) {integrals[i] = 0; targetsChanged[i] = true;}                 //setting all the flags in targetsChanged to signal a reset of the derivative and integral terms to avoid overcompensation
   
}

void changeTarget(int index, int newTarget)
{

  targets[index] = newTarget;
  integrals[index] = 0;
  targetsChanged[index] = true;                                                     //set the flag for a change of targets to reset the derivative and integral terms to avoid over compensastion
  switch(index)
  {

    case 0:
    lastErrors[index] = targets[index] - imu.anglePitch;
    break;
    
    case 1:
    lastErrors[index] = targets[index] - imu.angleRoll;
    break;

    case 2:
    lastErrors[index] = targets[index] - imu.heading;
    break;

    case 3:
    lastErrors[index] = targets[index] - imu.altitude;
    break;
 
  }
  
}

int *calibrateMotors()                                                                      //finds out the start up speeds of the individual start up voltages of the motors and
{
  
   float threshold;                                                                         //how large the magnitude of the vibrations can be until we decide that the motors are on
   imu.readMPUdataRaw();                                                                     //updating
   float mag = sqrt(sq(imu.accX) + sq(imu.accY));                                           //magnitude of the vibrations
   int newOffsets[4] = {0,0,0,0};                                                           
   int turnOnSpeeds[4] = {0,0,0,0};                                                         //will hold the voltages the motors turn on at
   int currentSpeed = 0;                                                                    //holds the current speed we are testing
   for(int i = 0; i < 4; i++)                                                               //will run through every motor
   {
      while(mag < threshold)                                                                //as long as the magnitude of the vibrations is small enough, we assume the motor isn't fully turned on
      {

        currentSpeed += 10;                                                                 
        motors.writeMotor(i, currentSpeed);                                                 //write motors more and more speed
        imu.readMPUdataRaw();                                                               //read the MPU data
        int mag = sqrt(sq(imu.accX) + sq(imu.accY));                                        //calculating magnitude of vibrations
        
      }

       turnOnSpeeds[i] = currentSpeed;                                                      //store speeds
       
   }
   int topSpeed = max(max(turnOnSpeeds[0], turnOnSpeeds[1]), max(turnOnSpeeds[2], turnOnSpeeds[3]));                                                        //take max start on speed, calculate offsets to make them all that speed
   for(int i = 4; i < 4; i++) {newOffsets[i] = topSpeed - turnOnSpeeds[i];}                 //calculating offsets
   motors.changeOffsets(newOffsets);                                                        //setting current offsets to the new ones
   return newOffsets;

}

