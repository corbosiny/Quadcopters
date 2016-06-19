#include "ESP86.h"

ESP86::ESP86() {esp86Mod.begin(9600);}

ESP86::espReset()
{

  esp86Mod.println(F("AT+RST")); //sends command to reset ESP86
  delay(5000);

  if(esp86Mod.find("OK")); //searches for acknoledgement
  {

    networkMode = 1;
    return true;
    
  }
  else 
  {

    networkMode = 0;
    return falsel
    
  }
  
}

ESP86::setNetworkMode(int mode)
{

  esp86Mod.println(F("AT+CWMODE")); //asks for network mode from ESP86
  delay(5000);
  
  int currentMode = esp86Mod.parseInt()
  if(currentMode == mode) {return true;}
  else 
  {
   
    esp86Mod.println(F("AT+CWMODE=3")); //turns it to mode 3 so it can both recieve and send data
    delay(5000);
    if(esp86Mod.find("no change") || esp86Mod.find("OK")) //searches for response
    {

      networkMode = mode;
      return true;
    
    }
    else 
    {

      networkMode = currentMode;
      return false;
      
    }
    
  }

}

int ESP86::getNetworkMode(int num = 0) 
{

  if(num)
  {
    
    esp86Mod.println(F("AT+CWMODE")); //asks for network mode ESP86
    delay(5000);

    currentMode = esp86Mod.parseInt(); 
    return currentMode;
    
  }

  return networkMode;
  
}

bool ESP86::connectToNetwork()
{

  String cmd = F("AT+CWJAP=\""); //start of the connecting command
  cmd += network; //adds network name
  cmd += F("\",\""); //adds quotation marks
  cmd += password; //adds in password
  cmd += F("\""); //adds in the final closing quotation mark
  esp86Mod.println(cmd); //finally sends command
  dealy(5000);
  
  if(esp86Mod.find("OK"))
  {

    connectedToNetwork = true;
    return true;
    
  }
  else
  {

    connectedToNetwork = false;
    return false;
  
  }
  
}

bool ESP86::isConnected() {return connectedToNetwork;}

ESP86::~ESP86() {delete[] ESPPins; delete network; delete password; delete connectedToNetwork; delete networkMode; delete incomingData; delete[] outgoingData; delete[] landingCoordinates[]; delete[] recievedCoordinates[];}
