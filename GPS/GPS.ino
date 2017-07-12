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

boolean readInGPSbuffer()
{
  if(gpsSerial.available())
  {

      while (gpsSerial.available())
      {
        int c = gpsSerial.read();
        if(gps.encode(c)) {return true;}
        else {return false;}
      }
    
  }
  else {return false;} 
}

void getGPSData(boolean getDatetime = false)
{
  gps.get_position(&lat, &lon, &lastFix);
  if(getDatetime) {gps.get_datetime(&date, &time, &lastFix);}
  speed = gps.speed();
  course = gps.course();
}

void updateGPSData()
{
  while(!readInGPSbuffer())
  {;}
  getGPSData();
}



int calcDistanceBetweenTwoPoints(int point1[2], int point2[2])    //point1 = currentPoint, point2 = point in question
{
  int yVector = point2[0] - point1[0];
  int xVector = point2[1] - point1[1];
  int distance = sqrt(sq(xVector) + sq(yVector));
  return distance;
}


int calcAngleBetweenTwoPoints(int point1[2], int point2[2])
{
  int yVector = point2[0] - point1[0];
  int xVector = point2[1] - point1[1];
  int angle = atan2(yVector ,xVector);
  return angle;
}

int *calcPolarHeadingBetweenTwoPoints(int point1[2], int point2[2])
{
  int heading[2];
  heading[0] = calcDistanceBetweenTwoPoints(point1, point2);
  heading[1] = calcAngleBetweenTwoPoints(point1, point2);
  return heading;
}

