#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

int rxPin, txPin;
SoftwareSerial gpsSerial(rxPin, txPin);
Adafruit_GPS GPS(&gpsSerial);
float distanceTol = 3; //meters

#include <PID.h>
#include <IMU.h>
#include <MotorController.h>

int motors[] = {1,2,3,4};
int constants[] = {1,2,3};
int maxAdjust = 45;  //degrees 
PIDcontroller pids(motors, constants, 40);

void setup() 
{

  GPS.begin(9600);
  gpsSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

}

void loop() 
{

  for(int i = 0; i < 4; i++) {pids.adjustAxis(i);}
  updateGPS();
  int examplePoint[2] = {0, 0};
  float headingDiff = calcRelationHeading(examplePoint);
  if(headingDiff != -1)
  {
    float pitchAdjust = sin(headingDiff) * maxAdjust;
    float rollAdjust = cos(headingDiff) * maxAdjust;

    pids.changeTarget(0, pitchAdjust);
    pids.changeTarget(1, rollAdjust);
  }
  
}

void clearSerialBuffer() 
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
  if(latDifference < distanceTol && lonDifference < distanceTol) {return -1;}
  int distanceMag = sqrt(sq(latDifference) + sq(lonDifference));
  int angleRelation = atan(lonDifference/latDifference);
  angleRelation *= 180 / PI;
  if(angleRelation < 0 && latDifference < 0) {angleRelation += 180;} 
  return angleRelation;

}

