//   @@@@@@@@@@@@@@
//  @              @
// @                @
// @                @
//@                @ 
//@          @@  @
//@        @
//@       @         - Ｍｏｎｅｙ, programming beast
//@        @
//@          @@  @
//@                @
// @                @   
// @                @
//  @              @
//   @@@@@@@@@@@@@@
//      

//GPS library for controlling the GPS
//and extracting the data you need qucikly from it
//subteam: Corey Hulse

//GPS: EM506(or any tinyGPS compatable GPS)
//Pin1: GND
//Pin2: Vin(5v)
//Pin3: Rx, Pin 4 on Arduino
//Pin4: TX, Pin 3 on Arduino
//Pin5: GND
//Pin6: 1PPS, Not connected
#include "GPS.h"

GPS::GPS(int rxPin, int txPin)
{

   this->rxPin = rxPin;
   this->txPin = txPin;
  
  gpsSerial = new SoftwareSerial(rxPin, txPin);
  gpsSerial->begin(4800);
  
}

void GPS::updateGPSData()
{
  while(!readInGPSbuffer())
  {
    ;
  }
  parseGPSData();
}

boolean GPS::readInGPSbuffer()
{
  if(gpsSerial->available())
  {

      while (gpsSerial->available())
      {
        int c = gpsSerial->read();
        if(gps.encode(c)) {return true;}
        else {return false;}
      }
    
  }
  else {return false;} 
}

void GPS::parseGPSData()
{
  gps.get_position(&latitude, &longitude, &lastFix);
  gps.get_datetime(&date, &time, &lastFix);
  speed = gps.speed();
  course = gps.course();
}

int GPS::calcDistanceBetweenTwoPoints(int point1[2], int point2[2])    //point1 = currentPoint, point2 = point in question
{
  int yVector = point2[0] - point1[0];
  int xVector = point2[1] - point1[1];
  int distance = sqrt(sq(xVector) + sq(yVector));
  return distance;
}


int GPS::calcAngleBetweenTwoPoints(int point1[2], int point2[2])
{
  int yVector = point2[0] - point1[0];
  int xVector = point2[1] - point1[1];
  int angle = atan2(yVector, xVector);
  return angle; 
}

int *GPS::calcPolarHeadingBetweenTwoPoints(int point1[2], int point2[2])
{
  int heading[2];
  heading[0] = calcDistanceBetweenTwoPoints(point1, point2);
  heading[1] = calcAngleBetweenTwoPoints(point1, point2);
  return heading;
}

int GPS::getLatitude() {return latitude;}
int GPS::getLongitude() {return longitude;}
