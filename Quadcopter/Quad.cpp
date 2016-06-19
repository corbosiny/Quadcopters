#include "Quad.h"

Quad::Quad(int p_motorPins[], int p_accelerometerPins[], int p_sensorPins[], int p_ESPPins[])
{
 
    MotorController motorController(p_motorPins);
    
    Balancer balancer(p_accelerometerPins);
    
    ObstacleAvoider obstacleAvoider(p_sensorPins)

    PositionTracker positionTracker();

    ESP86 esp(p_ESPPins);
    
  for(int i = 0; i < sizeof(p_motorPins) / sizeof(p_motorPins[0]); i++)
  {

    pinMode(p_motorPins[i], OUTPUT);
    digitalWrite(p_motorPins[i], OUTPUT);

    pinMode(p_accelerometerPins[i], INPUT);
    pinMode(p_sensorPins[i], INPUT);
    
  }
    
}

Quad::Quad(int p_motorPins[4], int p_accelerometerPins[4], int p_sensorPins[4], int p_ESPPins[], int p_tolerances[3])
{

  MotorController motorController(p_motorPins);
    
  Balancer balancer(p_accelerometerPins);
    
  ObstacleAvoider obstacleAvoider(p_sensorPins)

  PositionTracker positionTracker(p_tolerances);
    
  for(int i = 0; i < sizeof(p_motorPins) / sizeof(p_motorPins[0]; i++)
  {

    pinMode(p_motorPins[i], OUTPUT);
    digitalWrite(p_motorPins[i], OUTPUT);

    pinMode(p_accelerometerPins[i], INPUT);
    pinMode(p_sensorPins[i], INPUT);
    
  }
  
}

void Quad::setSensorPin(int sensorNum, int newPin) {obstacleAvoider.sensorPins[sensorNum - 1] = newPin;}
int Quad::getSensorPin(int sensorNum) {return obstacleAvoider.sensorPins[sensorNum - 1];}

void Quad::setMotorPin(int motorNum, int newPin) {motorController.motorPins[motorNum - 1] = newPin;}
int Quad::getMotorPin(int motorNum) {return motorController.motorPins[motorNum - 1];}

int Quad::readMotor(int motorNum) {return analogRead(motorController.motorPins[motorNum - 1]);}

void Balancer::setAccelerometerPin(int numAccel, int newPin) {balancer.accelerometerPins[numAccel - 1] = newPin;}
int Balancer::getAccelerometerPin(int numAccel) {return balancer.accelerometerPins[numAccel - 1];}

void Quad::setAdjustSpeed(int adjustSpeed, int newSpeed) {motorController.adjustSpeeds[adjustSpeed - 1] = newSpeed;}
int Quad::getAdjustSpeed(int adjustSpeed) {return motorController.adjustSpeeds[adjustSpeed - 1];}

void Quad::setTolerance(int axis, int newTolerance) {positionTracker.tolerances[axis - 1] = newTolerance;}
void Quad::setAllTolerances(int newTolerance) 
{

   for(int i = 0; i < sizeof(positionTracker.tolerances) / sizeof(positionTracker.tolerances[0]); i ++) {positionTracker.tolerances[i] = newTolerance;}
  
}
int Quad::getTolerance(int axis) {return obstacleAvoider.tolerances[axis - 1];}

void Quad::setMaxSpeed(int newMaxSpeed) {motorController.MAX_SPEED = newMaxSpeed;}
int Quad::getMaxSpeed() {return motorController.MAX_SPEED;}

void Quad::setCruisingSpeed(int newCruisingSpeed) {motorController.CRUISING_SPEED = newCruisingSpeed;}
int Quad::getCruisingSpeed() {return motorController.CRUISING_SPEED;}

void Quad::setBatteryThreshold(int newThresh) {BATTERY_THRESHOLD = newThresh;}
int Quad::getBatteryThreshold() {return BATTERY_THRESHOLD;}

void Quad::setBatteryPin(int newPin) {batteryPin = newPin;}
int Quad::getBatteryPin() {return batteryPin;}
float Quad::readBattery() {return analogRead(batteryPin);}

void Quad::setESPPin(int numPin, int newPin) {ESPPins[numPin] = newPin;} //1 = tx, 2 = rx
int Quad::getESPin(int numPin) {return ESPPins[numPin];} //1 = tx, 2 = rx

void setNetworkName(String newName) {esp.network = newName;}
String getNetworkName() {return esp.network;}

void setNetworkPassword(String newPassword) {esp.password = newPassword;}
String getNetworkPassword() {return esp.password;}

void Quad::setPosition(int axis, int newPosition) {positionTracker.positions[axis - 1] = newPosition;} //allows the user to recalibrate the quad copters relative position, NOT RECOMMENDED TO USE EVER, I MEAN IT, DON'T CALL THIS MILES WHILE TESTING!
void Quad::setAllPositions(int p_position) {for(int i = 0; i < sizeof(p_positions) / sizeof(p_positions[0]); i++){positionTracker.positions[i] = p_position;}} //allows the user to recalibrate the quad copters relative position, NOT RECOMMENDED TO USE EVER, I MEAN IT, DON'T CALL THIS MILES WHILE TESTING!
void Quad::setAllPositions(int p_positions[]) {memcpy(positionTracker.positions, p_positions);} //allows the user to recalibrate the quad copters relative position, NOT RECOMMENDED TO USE EVER, I MEAN IT, DON'T CALL THIS MILES WHILE TESTING!
int Quad::getPosition(int axis) {return positionTracker.positions[axis - 1];}

void Quad::manualMode() 
{

  int userInput[5] = {1, 0, 0 ,0 ,0};
  int userSpeeds[4];
   
  motorController.stopAllMovement();
  
  while(userInput[0] != 0) 
  {

    //will add function to read user commands remotely here
  
    for(int i = 0; i < 4; i++) {userSpeeds[i] = userInput[i++];}   

    motorController.writeAllMotors(userSpeeds);
    
  }

  motorController.stopAllMovement();
 
}

Quad::~Quad() {delete motorController; delete balancer; delete obstacleAvoider; delete positionTracker; delete BATTERY_THRESHOLD; delete batteryPin;}
