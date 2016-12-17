#include <Servo.h>

int motorPins[] = {};
int motorSpeedOffsets[] = {0,0,0,0};
int currentMotorSpeeds[] = {0, 0, 0, 0};

Servo motors[4];
void setup() 
{

  for(int i = 0; i < 4; i++) {motors[i].attach(motorPins[i]);}
  
}

void loop() 
{

}

void writeMotor(int motorNumber, int newSpeed)
{
  currentMotorSpeeds[motorNumber - 1] = newSpeed - motorSpeedOffsets[motorNumber - 1];
  motors[motorNumber - 1].writeMicroseconds(currentMotorSpeeds[motorNumber - 1]);
}

void writeMotors(int speeds[])
{

  for(int i = 0; i < 4; i++) 
  {
    motors[i].writeMicroseconds(speeds[i] - motorSpeedOffsets[i]); 
    currentMotorSpeeds[i] = speeds[i] - motorSpeedOffsets[i];
  }
  
}

void adjustMotors(int adjustments[])
{

   for(int i = 0; i < 4; i++) {currentMotorSpeeds[i] += adjustments[i];}
   writeMotors(currentMotorSpeeds);

}

void adjustMotor(int motorNumber, int adjustment) {writeMotor(motorNumber, currentMotorSpeeds[motorNumber - 1] + adjustment);}

void motorTest()
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

