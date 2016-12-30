//
//Feromone Robotics Qaudcopter main flight controller
//This controller is responsible for calculating the change of states needed in order to
//fly toward a certain point and feeds those states to the PIDS
//Coders: Corey Hulse
//***See flight system manual for a description of all the other systems***
//GPS = Adafruit Ultimate GPS module
//rxPin and txPin to their respecitve pins on the GPS
//Vin - 3 to 5 volts
//GND - GND
//Don't worry about the rest of the pins
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

int rxPin, txPin;                                                         //setting up a serial connection with the GPS module
SoftwareSerial gpsSerial(rxPin, txPin);
Adafruit_GPS GPS(&gpsSerial);
float distanceTol = 3; //meters

#include <PID.h>                                                          //including all our system libraries
#include <IMU.h>
#include <MotorController.h>

int motors[] = {1,2,3,4};                                                 //motorpins
int constants[] = {1,2,3};                                                //constants for the PIDS
int maxAdjust = 40;  //degrees                                            //maximum amount the drone will adjust to one side, very important to choose this carefully
int seaLevelPress = 30.2                                                  //local air pressure at sea level, for barometer, 30.2 is for mesa Arizona
PIDcontroller pids(motors, constants, seaLevelPress);                     //initializing the PIDS which in turn initialize the motorController and IMU

void setup() 
{

  GPS.begin(9600);                                                        //kickstarting our GPS
  gpsSerial.begin(9600);
  GPS.sendCommand("$PCGMD,33,0*6D");
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

}

void loop() 
{

  for(int i = 0; i < 4; i++) {pids.adjustAxis(i);}                      //first we start off by adjusting each axis with the IMU as staying in the air is the #1 priority
  updateGPS();
  int examplePoint[2] = {0, 0};                                         //use to test GPS, put in whatever here to test accuracy
  float headingDiff = calcRelationHeading(examplePoint);                //calculates distance and angle to example point
  if(headingDiff != -1)                                                 //headingDiff returns -1 if we are withing a certain distance of our point and do not need to move any more
  {
    float pitchAdjust = sin(headingDiff) * maxAdjust;                   //splits the maxAdjust between each axis to get the resultant movement as if we had rotated to and were moving directly toward the point
    float rollAdjust = cos(headingDiff) * maxAdjust;                    //using trig functions means no one axis goes over the maximum adjust

    pids.changeTarget(0, pitchAdjust);                                  //change the target state of each axis in the PIDS to reflect the rotation we wish to achieve so we can stay in the desired rotation and fly in the desired direction
    pids.changeTarget(1, rollAdjust);
  }
  
}

void clearSerialBuffer()                                                //see adafruitGPS code for details on all the GPS code below
{
  if(GPS.newNMEAreceived()) {GPS.parse(GPS.lastNMEA());}
}

void updateGPS()
{

  clearSerialBuffer();
  while(!GPS.newNMEAreceived()) {}
  GPS.parse(GPS.lastNMEA());
  
  while(!GPS.newNMEAreceived()) {}
  GPS.parse(GPS.lastNMEA());
  
}

float calcRelationHeading(int point[2]) //0 = lattitude, 1 = longitude
{

  int latDifference = point[0] - GPS.latitude;
  int lonDifference = point[1] - GPS.longitude;
  if(latDifference < distanceTol && lonDifference < distanceTol) {return -1;} //if we are within our tolerance than we do not need to move toward our point anymore so we return -1 as a flag to show this
  int distanceMag = sqrt(sq(latDifference) + sq(lonDifference));
  int angleRelation = atan(lonDifference/latDifference);
  angleRelation *= 180 / PI;
  if(angleRelation < 0 && latDifference < 0) {angleRelation += 180;} 
  return angleRelation;

}

