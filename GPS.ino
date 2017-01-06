#include <SoftwareSerial.h>
#include <TinyGPS.h>


//****NOTE THIS CODE IS OBSOLETE NOW THAT WE ARE USING THE ADAFRUIT CODE****

TinyGPS gps;  //we are using the tiny GPS library as it is very convienent

int rxPin, txPin;       //used for talking with the GPS, can be any two pins
long int lat, lon;      //latitude and longitude
unsigned long time, date, course, speed, lastFix;                                 //most of these are not used but could be implemented for extra sensing capabilites

float axisAdjust;                                                                 

SoftwareSerial gpsSerial(rxPin, txPin);         //intializes the pseudo serial port between the arduino and the gps

void setup() 
{



}

void loop() 
{



}

boolean updateGPS()          //reads available GPS data if any
{

  if(gpsSerial.available())
  {

      while (gpsSerial.available())  //keeps reading until a valid statement is read
      {
        char c = gpsSerial.read();
        if(gps.encode(c)) {return true;}
      }
    
  }
  else {return false;} //signals there was no data
  
}

void getGPSData(boolean updateFirst = True, boolean getDatetime = false) //reads in all the values parsed from the GPS statement
{

  if(updateFirst)   //defaulted to always update the data before reading it
  {while(!updateGPS());}    //wait for a valid update

  gps.get_position(&lat, &lon, &lastFix); //we dont use the float version of this method as it takes up a very large amount of memory
  if(getDatetime) {gps.get_datetime(&date, &time, &lastFix);}
  speed = gps.speed();
  course = gps.course();

}

int calcDistance(int point1[2], int point2[2])    //point1 = currentPoint, point2 = point in question
{                                                 //calculates the magnitude of distance between two points

  int y = point2[0] - point1[0];
  int x = point2[1] - point1[1];
  int distance = sqrt(sq(x) + sq(y));
  return distance;

}


int calcAngle(int point1[2], int point2[2])     //calculates angle of the vector in realtion to its current point using arc tan
{

  int y = point2[0] - point1[0];
  int x = point2[1] - point1[1];
  float angle = atan(y / x);
  if(x < 0) {angle += 180}; //used because atan cant tell from a negative answer which term is negative, so if x is negative we add 180 to get the correct angle
  return angle;
  
}

int *calcRelation(int point1[2], int point2[2]) //calculates magnitude of distance and angle of distance vector between two points
{

  int relation[2];
  relation[0] = calcDistance(point1, point2);
  relation[1] = calcAngle(point1, point2);
  return relation;
  
}

