#ifndef PositionTracker_h
#define PositionTracker_h

#include "Arduino.h"

class PositionTracker 
{

  friend class Quad; //allows Quad to access private variables
  
  public:
  PositionTracker();
  PositionTracker(int p_tolerances[]);
  
  float calcTimeInterval(int startTime); 
  float calcAcceleration(int axis);
  float calcVelocity(int axisAcceleration, int timeInterval);
  float calcPosition(int axis, int velocity, int timeInterval); //tracks its given relative coordinates on three axises

  void follow(); //alligns itself within a given tolerance of the users axis

  void allignAxis(int axis, int distance); //user can call this to allign the robot within a given tolerance on an axis, this is only temporary

  void headToPoint(int coordinates[]);
  void returnHome(); //returns to the charging pad by user command or when the battery charge drops too low

  ~PositionTracker();

  private:
  int startingTime = 0;

  int initialPosition = 0; 
  bool movingAxis[3] = {false, false, false}; //used in the moving axis function that uses distance as a paramter, do not confuse with moving 
  
  float velocities[3] = {0,0,0};
  float positions[3] = {0,0,0};
  float userPositions[3] = {0,0,0};
  
  int tolerances[3]; //the tolerance the robot will align on the axises, first is the X axis, second is Y, third is Z

  int padPosition = [0,0,0]; //position of the charging pad

  void allignAxis(int axis, int num = 0); //the robot uses this to allign itself within a given tolerance on whatever axis entered, if num does not equal one then it lines up exaclty even
  
  void allignAllAxises(int num = 0); //alligns all the axises within a given tolerance, if num does not equal one then the axises are lined up exactly

};

#endif
