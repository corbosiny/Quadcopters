#ifndef ESP86_h
#define ESP86_h

#include "Arduino.h"
#include <SoftwareSerial.h>

class ESP86
{

  friend class Quad; //allows Quad to access private variables
  
  public:
  ESP86();

  bool espReset();
  
  bool setNetworkMode(int mode);
  int getNetworkMode(int num = 0;);

  bool connectToNetwork();
  bool connectedToWifi();
  bool isConnected();
  
  bool broadCastData(int data);
  bool broadCastData(int data[]);
  //bool requestLanding(int landingCoordinates[]);

  ~ESP86(); //Deconstructor
  
  private:
  SoftwareSerial esp86Mod(10, 11); //Defines TX and RX
  
  int ESPPins[];
  
  String network;
  String password;
  
  boolean connectedToNetwork = false;
  int networkMode;
  
  int outgoingData[];
  int incomingData[];
  
  int landingCoordinates[];
  int recievedCoordinates[];

}


#endif
