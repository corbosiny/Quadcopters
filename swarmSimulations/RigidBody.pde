class RigidBody    //Essentially a rigid body object of the Drone
{
  
  static final int NUM_AXIS = 2;                                            
  
  float PIDconstants[] = new float[3];                                      //0 = proportional, 1 = integral, 2 = derivative
  int maxPIDoutputs[] = null;                                               
  int coordinates[] = new int[NUM_AXIS];                                    
  float outsideForces[] = new float[NUM_AXIS];                              
  float integral[] = new float[NUM_AXIS];                                   //holds our running integral calculations for the PID controller
  float lastErrors[] = {0, 0};                                              //holds the previous error term for derivatives
  boolean desiredStateReset = false;                                        //lets us know when a new desired state is set for the PID so we can canel out derivative error spikes
  int desiredState[] = {0, 0};                                              //the state the RigidBody is trying to achieve
  float lastMeasurmentTimes[] = new float[NUM_AXIS];                                    
  float timeSinceLastUpdate;                                                              
  color Color;                                                              
  
  RigidBody(int coordinates[])
  {
    
    //use arrayCopy to make deep copies
    arrayCopy(coordinates, this.coordinates);    
    arrayCopy(coordinates, this.desiredState);                                                                    //setting our intial desired state to the spawn coordiantes, this can be changed
    
    this.Color = color(random(0, 255), random(0, 255), random(0, 255));
    
    for(int i = 0; i < NUM_AXIS; i++) {lastMeasurmentTimes[i] = millis(); integral[i] = 0; outsideForces[i] = 0;}    //initializing our integral terms, and setting forces to 0
    for(int i = 0; i < 3; i++) {PIDconstants[i] = 0;}                                                                
    
    rigidBodies = (RigidBody[])append(rigidBodies, this);                                            
  }
  
  RigidBody(int coordinates[], float outsideForces[])
  {
    //use arrayCopy to make deep copies
    arrayCopy(coordinates, this.coordinates);                                                                     
    arrayCopy(outsideForces, this.outsideForces);
    arrayCopy(coordinates, this.desiredState);                                                                    //setting our intial desired state to the spawn coordiantes, this can be changed
    
    this.Color = color(random(0, 255), random(0, 255), random(0, 255));
    
    for(int i = 0; i < NUM_AXIS; i++) {lastMeasurmentTimes[i] = millis(); integral[i] = 0;}                           //starting the timer for our measurment timeSinceLastUpdates and setting our integral sums to zero as PID adjustment hasn't started
    for(int i = 0; i < 3; i++) {PIDconstants[i] = 0;}                                                             //if no constants are passed we have to set them to zero
    rigidBodies = (RigidBody[])append(rigidBodies, this);                                                                       
  }
  
  RigidBody(int coordinates[], float outsideForces[], float PIDconstants[], int maxPIDoutputs[])
  {
    //use arrayCopy for deep copies
    arrayCopy(coordinates, this.coordinates);
    arrayCopy(outsideForces, this.outsideForces);
    arrayCopy(PIDconstants, this.PIDconstants);
    arrayCopy(coordinates, this.desiredState);
    this.maxPIDoutputs = new int[3];
    arrayCopy(maxPIDoutputs, this.maxPIDoutputs);                                                                
    this.Color = color(random(0,255), random(0,255), random(0,255));
    
    for(int i = 0; i < NUM_AXIS; i++) {lastMeasurmentTimes[i] = millis(); integral[i] = 0;}                          //starting the timer for our measurment timeSinceLastUpdates and setting our integral sums to zero as PID adjustment hasn't started
    rigidBodies = (RigidBody[])append(rigidBodies, this);                                                     
  }
  
  RigidBody(int coordinates[], float outsideForces[], float PIDconstants[], int maxPIDoutputs[], color Color)
  {
    //use arrayCopy for deep copies
    arrayCopy(coordinates, this.coordinates);
    arrayCopy(outsideForces, this.outsideForces);
    arrayCopy(PIDconstants, this.PIDconstants);
    arrayCopy(coordinates, this.desiredState);
    this.maxPIDoutputs = new int[3];
    arrayCopy(maxPIDoutputs, this.maxPIDoutputs);
    this.Color = Color;
    
    for(int i = 0; i < NUM_AXIS; i++) {lastMeasurmentTimes[i] = millis(); integral[i] = 0;}                          //starting the timer for our measurment timeSinceLastUpdates and setting our integral sums to zero as PID adjustment hasn't started
    rigidBodies = (RigidBody[])append(rigidBodies, this);                                                       
  }
  
  float calcStateError(int num) {return desiredState[num] - coordinates[num];}                                   //calculates the error on one axis from its desired state to its current state
  
  float calcPIDAdjust(int axis)                                                                                  //runs through PID equations to determine its output to reach the desired state
  {
    
       timeSinceLastUpdate = (millis() - lastMeasurmentTimes[axis]) / 1000;                                                        //calculates change in time, converts from milliseconds to seconds
       float error = calcStateError(axis);                                                                  
       if(desiredStateReset == true) {lastErrors[axis] = error; desiredStateReset = false;}                      //checks if a new sate was set so we are resetting the reset to avoid falsely perceived error
        
       float proportional = PIDconstants[0] * error;
       
       integral[axis] += PIDconstants[1] * error * timeSinceLastUpdate;
       
       float derivative = PIDconstants[2] * (error - lastErrors[axis]) / timeSinceLastUpdate;
       
       lastErrors[axis] = error;                                                                                
       lastMeasurmentTimes[axis] = millis();                                                                        
       
       if(maxPIDoutputs != null)                                                                                      //if maxOutputs are entered then we regulate the output of our PID controller
       {
         if(proportional > maxPIDoutputs[0]) {proportional = maxPIDoutputs[0];}                                       //regulating max output in both positive and negative cases
         else if(abs(proportional) > maxPIDoutputs[0]) {proportional = maxPIDoutputs[0] * -1;}
         
         if(integral[axis] > maxPIDoutputs[1]) {integral[axis] = maxPIDoutputs[1];}                                   
         else if(abs(integral[axis]) > maxPIDoutputs[1]) {integral[axis] = maxPIDoutputs[1] * -1;}
         
         if(derivative > maxPIDoutputs[2]) {derivative = maxPIDoutputs[2];}                                           
         else if(abs(derivative) > maxPIDoutputs[2]) {derivative = maxPIDoutputs[2] * -1;}
       }
       
       float offset = integral[axis] + proportional + derivative;                                               
       return offset;
  
  }
 
 
  void changePIDConstants(float newConstants[]) {arrayCopy(newConstants, this.PIDconstants);}                  
 
  void applyForces()  //calculates the state of the next frame due to forces and PID outputs
  {

      for(int i = 0; i < NUM_AXIS; i++)  //going through each axis
      {
        
          float output = (calcPIDAdjust(i) + outsideForces[i]) * timeSinceLastUpdate;                                                
          
          for(int j = 0; j < rigidBodies.length; j++)                                                                //going through all rigidBodies in the simulation and checking if they are getting to close
          {
           if(rigidBodies[j] == this) {continue;}                                                                    //skip avoiding itself
           output += calcObstacleAvoidanceAdjust(i, rigidBodies[j]) * timeSinceLastUpdate;                    
          }
          
          if(abs(output) > (maxPIDoutputs[0] + maxPIDoutputs[1] - maxPIDoutputs[2]) * timeSinceLastUpdate) {output = (maxPIDoutputs[0] + maxPIDoutputs[1] - maxPIDoutputs[2]) * timeSinceLastUpdate * (output / abs(output));} //derivative is almost always going against the other terms, hence the subtraction
          coordinates[i] += output; //adding that to our state
      
      }
    
      
    //drawing our new ellipse
    fill(Color);
    ellipse(coordinates[0], coordinates[1], 10, 10);
    
  }
 
 
  float calcDistance(int x, int y)                                                                     //returns distance between two points
  {return sqrt((x - coordinates[0])^2 + (y - coordinates[1])^2);}
 
  
   float calcObstacleAvoidanceAdjust(int axis, RigidBody RigidBody2)                                                     //returns an adjustment force to help the drone avoid running into obstacles
   {
     float difference = coordinates[axis] - RigidBody2.coordinates[axis];                                  //how far the drone and object are from eachother, NOTE IT CAN BE NEGATIVE
     for(int i = 0; i < NUM_AXIS; i++) {if(i == axis) {continue;} if(abs(coordinates[i] - RigidBody2.coordinates[i]) > minDistance) {return 0;}} //checking if it is actually lined up on the other axes as well, if not then we know they aren't actually close
     
     if(abs(difference) > maxDistance) {return 0;}                                                     //if they are farther than the maxDistance we start avoiding at, then return no adjustment
     //integral[axis] -= timeSinceLastUpdate * this.constants[1] * difference;
     float totalForce = maxPIDoutputs[0] + maxPIDoutputs[1] - maxPIDoutputs[2];                                 //the max adjust we can use to avoid obstacles
     float mapped = map(abs(difference), maxDistance, minDistance, 0, totalForce);                     //map a scale from the distance we start avoiding at to the min proximity allowed to a scale of zero to the max adjust force
     if(abs(difference) > 0) {mapped *= (difference / abs(difference));}                               //here we preserver the sign of the adjustment, AKA make sure we adjust in the right direction, we check for zero to avoid divide by zero errors

     if(Float.isNaN(mapped))                                                                           //occasionally, due to how the map function works, we get NaN. We just return 0 as usually this means the difference value wasn't far in the range of our map scales I think
     {println("GOT YOUR NAN RIGHT HERE!"); return 0;}
     else if(abs(mapped) > totalForce)                                                                 //makes sure mapped stays within our bounds and preserves the sign when limiting the output
     {
       mapped = totalForce;
       if(abs(difference) > 0) {mapped *= (difference / abs(difference));}                             //divinding a number by the absolute value of itself is a way of getting the sign of that number
       else if(abs(calcStateError(axis))> 0) {mapped *= abs(calcStateError(axis)) / (calcStateError(axis));} 
     }
     
     return mapped;
   } 
  
}