#ifndef GPS_h
#define GPS_h

#include "Arduino.h"
#include <SoftwareSerial.h>
#include <TinyGPS.h>

class GPS
{

  public:
  GPS(int rxPin, int txPin);
  void updateGPSData();
  boolean readInGPSbuffer();
  void parseGPSData();
  int calcDistanceBetweenTwoPoints(int point1[2], int point2[2]);
  int calcAngleBetweenTwoPoints(int point1[2], int point2[2]);
  int *calcPolarHeadingBetweenTwoPoints(int point1[2], int point2[2]);
  int getLatitude();
  int getLongitude();
  
  private:
  TinyGPS gps;
  SoftwareSerial *gpsSerial = NULL;
  int rxPin, txPin;
  long int latitude, longitude;
  unsigned long time, date, course, speed, lastFix;
  float axisAdjust;
  
};


#endif
