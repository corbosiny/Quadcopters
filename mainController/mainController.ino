//Feromone Robotics Qaudcopter main flight controller
//This controller is responsible for calculating the change of states needed in order to
//fly toward a certain point and feeds those states to the PIDS
//Developers: Corey Hulse
//***See flight system manual for a description of all the other systems***
//GPS = Adafruit Ultimate GPS module
//rxPin and txPin to their respecitve pins on the GPS
//Vin - 3 to 5 volts
//GND - GND
//Don't worry about the rest of the pins on the GPS

#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

int rxPin, txPin;
SoftwareSerial gpsSerial(rxPin, txPin);
Adafruit_GPS GPS(&gpsSerial);
float distanceTol = 3; //meters

#include "PID.h"
#include "IMU.h"
#include "MotorController.h"
#include "oAvoider.h"

int motors[] = {1,2,3,4};
int constants[] = {5,5,.4};
int maxOutputs[] = {255, 255, 255};
int maxAdjust = 45;  //degrees
int seaLevelPressure = 40;
int maxDistance = 30;
int minDistance = 15;
int sensorPins[4][2] = {{11,10}, {9,8}, {7,6}, {5,4}};

//building up and initizilzing our drone systems
IMU newIMU(seaLevelPressure); 
MotorController motorC(motors);
ObstacleAvoider oAvoider(sensorPins, maxDistance, minDistance);
PIDcontroller pids(&motorC, &newIMU, constants, maxOutputs, &oAvoider);

void setup() 
{

  GPS.begin(9600);
  gpsSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

}

void loop() 
{

  for(int i = 0; i < 4; i++) {pids.adjustAxis(i);}              //adjusting through every dimension to stabalize ourselves
  updateGPS();                                                  //getting up to date GPS readings before planning flight paths
  int examplePoint[2] = {0, 0};
  float headingDiff = calcRelationHeading(examplePoint);        //polar coordinates to our desired location
  if(headingDiff != -1)
  {
    float pitchAdjust = sin(headingDiff) * maxAdjust;           //breaking up our movements in x and y into pitch and roll movements
    float rollAdjust = cos(headingDiff) * maxAdjust;

    pids.changeTarget(0, pitchAdjust);                          //setting our PID controller to put the body into that position so it will move how we want it
    pids.changeTarget(1, rollAdjust);
  }
  
}

void clearSerialBuffer() 
{
  if(GPS.newNMEAreceived()) {GPS.parse(GPS.lastNMEA());}  //removes junk caught up in buffer between readings
}

void updateGPS()          //clears the buffer than returns two NMEA sentences, one for lat and one for long
{

  clearSerialBuffer(); 
  while(!GPS.newNMEAreceived()) {}
  GPS.parse(GPS.lastNMEA());
  
  while(!GPS.newNMEAreceived()) {}
  GPS.parse(GPS.lastNMEA());
  
}

float calcRelationHeading(int point[2]) //calculates the polar coordinates to get to a desired point, 0 index = lattitude, 1 index = longitude
{

  int latDifference = point[0] - GPS.latitude; 
  int lonDifference = point[1] - GPS.longitude;
  if(latDifference < distanceTol && lonDifference < distanceTol) {return -1;}
  int distanceMag = sqrt(sq(latDifference) + sq(lonDifference));
  int angleRelation = atan(lonDifference/latDifference);
  angleRelation *= 180 / PI;
  if(angleRelation < 0 && latDifference < 0) {angleRelation += 180;} 
  return angleRelation;

}

