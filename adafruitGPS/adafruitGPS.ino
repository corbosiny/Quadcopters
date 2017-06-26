#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

int rxPin, txPin;
SoftwareSerial gpsSerial(rxPin, txPin);
Adafruit_GPS GPS(&gpsSerial);
void setup() 
{

  Serial.begin(9600);
  GPS.begin(9600);
  gpsSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
}

void loop() 
{
 
  updateGPS();
  Serial.print("Latitude: ");
  Serial.println(GPS.latitude);
  Serial.print("Longitude: ");
  Serial.println(GPS.longitude);
  Serial.print("Speed: ");
  Serial.println(GPS.speed);
  
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
  Serial.println(GPS.lastNMEA());
  
  while(!GPS.newNMEAreceived()) {}
  GPS.parse(GPS.lastNMEA());
  Serial.println(GPS.lastNMEA());
  
}

float calcRelationHeading(int point[2]) //0 = lattitude, 1 = longitude
{

  int latDifference = point[0] - GPS.latitude;
  int lonDifference = point[1] - GPS.longitude;

  int distanceMag = sqrt(sq(latDifference) + sq(lonDifference));
  int angleRelation = atan(lonDifference/latDifference);
  angleRelation *= 180 / PI;
  if(angleRelation < 0 && latDifference < 0) {angleRelation += 180;} 
  return angleRelation;
}


