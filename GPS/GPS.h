#ifndef GPS_h
#define GPS_h

#include "Arduino.h"
#include <SoftwareSerial.h>
#include <TinyGPS.h>

class GPS
{

  public:
  GPS(int p_rxPin, int p_txPin);
  boolean updateGPS();
  void getGPSData(boolean updateFirst = false, boolean getDatetime = false);
  int calcDistance(int point1[2], int point2[2]);
  int calcAngle(int point1[2], int point2[2]);
  int *calcRelation(int point1[2], int point2[2]);
  
  private:
  TinyGPS gps;
  SoftwareSerial *gpsSerial = NULL;
  int rxPin, txPin;
  long int lat, lon;
  unsigned long time, date, course, speed, lastFix;
  float axisAdjust;
  
};


#endif
