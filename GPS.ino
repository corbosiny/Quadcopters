#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps; 

int rxPin, txPin;
long int lat, lon;
unsigned long time, date, course, speed, lastFix;

float axisAdjust;

SoftwareSerial gpsSerial(rxPin, txPin);

void setup() 
{



}

void loop() 
{



}

boolean updateGPS()
{

  if(gpsSerial.available())
  {

      while (gpsSerial.available())
      {
        int c = gpsSerial.read();
        if(gps.encode(c)) {return true;}
      }
    
  }
  else {return false;}
  
}

void getGPSData(boolean updateFirst = false, boolean getDatetime = false)
{

  if(updateFirst)
  {while(!updateGPS());}

  gps.get_position(&lat, &lon, &lastFix);
  if(getDatetime) {gps.get_datetime(&date, &time, &lastFix);}
  speed = gps.speed();
  course = gps.course();

}

int calcDistance(int point1[2], int point2[2])    //point1 = currentPoint, point2 = point in question
{

  int y = point2[0] - point1[0];
  int x = point2[1] - point1[1];
  int distance = sqrt(sq(x) + sq(y));
  return distance;

}


int calcAngle(int point1[2], int point2[2])
{

  int y = point2[0] - point1[0];
  int x = point2[1] - point1[1];
  int angle = atan(y / x);
  return angle;
  
}

int *calcRelation(int point1[2], int point2[2])
{

  int relation[2];
  relation[0] = calcDistance(point1, point2);
  relation[1] = calcAngle(point1, point2);
  return relation;
  
}

