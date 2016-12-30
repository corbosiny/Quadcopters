//Test code for reading GPS data and calculating vectors to other points
//Coder - Corey Hulse
//***See flight system manual for a description of all the other systems***
//GPS = Adafruit Ultimate GPS module
//rxPin and txPin to their respecitve pins on the GPS
//Vin - 3 to 5 volts
//GND - GND
//Don't worry about the rest of the pins

#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

int rxPin, txPin;                                                                          //used for talking with the GPS
SoftwareSerial gpsSerial(rxPin, txPin);                                                    //set up serial connection with GPS
Adafruit_GPS GPS(&gpsSerial);                                                              //create GPS object
void setup()                            
{

  Serial.begin(9600);             
  GPS.begin(9600);
  gpsSerial.begin(9600);
  GPS.sendCommand("$PCGMD,33,0*6D");
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);                                             //sets up GPS for only RMC and GGA commands
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);                                               //sets the gps for 10Hz refresh rate, may change depending on stability
  
}

void loop() 
{
 
  updateGPS();                                                                             //parses NMEA sentences and determines location, speed, heading, etc.
  Serial.print("Latitude: ");                                                              //just printing out values for testing
  Serial.println(GPS.latitude);                     
  Serial.print("Longitude: ");
  Serial.println(GPS.longitude);
  Serial.print("Speed: ");
  Serial.println(GPS.speed);
  
}

void clearSerialBuffer() 
{
  if(GPS.newNMEAreceived()) {GPS.parse(GPS.lastNMEA());}                                  //the buffer is constantly filling up so we clear it before reading so we have relavant data
}

void updateGPS()
{

  clearSerialBuffer();                                                                    //clear out buffer
  while(!GPS.newNMEAreceived()) {}                                                        //read in first NMEA, but one will be GGA/RCA and the next will be the other so we must wait
  GPS.parse(GPS.lastNMEA());
  Serial.println(GPS.lastNMEA());
  
  while(!GPS.newNMEAreceived()) {}                                                        //get the second statement in the pair
  GPS.parse(GPS.lastNMEA());
  Serial.println(GPS.lastNMEA()); 
  
  int examplePoint[2] = {0, 0};                                                           //used to test calcRelationHeading(), put random points here
  float headingRelation = calcRelationHeading();
  Serial.print("Heading Relation: ");
  Serial.println(headingRelation);
  
}

float calcRelationHeading(int point[2]) //0 = lattitude, 1 = longitude
{

  int latDifference = point[0] - GPS.latitude;                                            //calculates difference in lattitude
  int lonDifference = point[1] - GPS.longitude;                                           //calculates difference in longitude

  int distanceMag = sqrt(sq(latDifference) + sq(lonDifference));                          //uses distance formula to calculate the aprroximate distance
  Serial.print("Distance Mag: ");
  Serial.println(distanceMag);
  int angleRelation = atan(lonDifference/latDifference);                                  //the drone stays at 0 degrees heading so we calculate what angle the specified point is in relation to the drones heading
  angleRelation *= 180 / PI;                                                              //turn angle from radians into degrees
  if(angleRelation < 0 && latDifference < 0) {angleRelation += 180;}                      //if the angle is negative, but it is the x distance that is negative add 180 as atan can't determine whether y or x was negative in the calculation
  return angleRelation;                                                                   //return the calculated heading difference

}

