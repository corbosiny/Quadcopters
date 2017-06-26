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
//subteam: Corey Hulse(thats why I can get away with putting my logo up top)

//GPS: EM506(or any tinyGPS compatable GPS)
//Pin1: GND
//Pin2: Vin(5v)
//Pin3: Rx, Pin 4 on Arduino
//Pin4: TX, Pin 3 on Arduino
//Pin5: GND
//Pin6: 1PPS, Not connected
#include "GPS.h"

GPS::GPS(int p_rxPin, int p_txPin)
{

   rxPin = p_rxPin;
   txPin = p_txPin;
  
  gpsSerial = new SoftwareSerial(p_rxPin, p_txPin);
  gpsSerial->begin(4800);
  
}


boolean GPS::updateGPS()
{

  if(gpsSerial->available())
  {

      while (gpsSerial->available())
      {
        int c = gpsSerial->read();
        if(gps.encode(c)) {return true;}
      }
    
  }
  else {return false;}
  
}

void GPS::getGPSData(boolean updateFirst = false, boolean getDatetime = false)
{

  if(updateFirst)
  {while(!updateGPS());}

  gps.get_position(&lat, &lon, &lastFix);
  if(getDatetime) {gps.get_datetime(&date, &time, &lastFix);}
  speed = gps.speed();
  course = gps.course();

}

int GPS::calcDistance(int point1[2], int point2[2])    //point1 = currentPoint, point2 = point in question
{

  int y = point2[0] - point1[0];
  int x = point2[1] - point1[1];
  int distance = sqrt(sq(x) + sq(y));
  return distance;

}


int GPS::calcAngle(int point1[2], int point2[2])
{

  int y = point2[0] - point1[0];
  int x = point2[1] - point1[1];
  int angle = atan(y / x);
  return angle;
  
}

int *GPS::calcRelation(int point1[2], int point2[2])
{

  int relation[2];
  relation[0] = calcDistance(point1, point2);
  relation[1] = calcAngle(point1, point2);
  return relation;
  
}

