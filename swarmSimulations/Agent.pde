class Agent    //Essentially a rigid body object of the Drone
{
  
  static final int NUM_AXIS = 2;                                            //number of dimensions in the simulation
  
  float constants[] = new float[3];                                         //PID constants
  int maxOutputs[] = null;                                                  //limits speed the agents can go for each term in a PID controller
  int coordinates[] = new int[NUM_AXIS];                                    //current state of the agent
  float forces[] = new float[NUM_AXIS];                                     //holds the forces in each direction
  float integral[] = new float[NUM_AXIS];                                   //holds our running integral calculations for the PID controller
  float lastErrors[] = {0, 0};                                              //holds the previous error term for derivatives
  boolean reset = false;                                                    //lets us know when a new desireed state is set for the PID so we can canel out derivative error spikes
  int desiredState[] = {0, 0};                                              //the state the agent is trying to achieve
  float lastMeasurments[] = {0, 0};                                         //last time we took measurments at used for time based calculations like derivative and integral
  float clock;                                                              //holds the time between force calculations
  color Color;                                                              
  
  Agent(int coordinates[])
  {
    
    //use arrayCopy to make deep copies
    arrayCopy(coordinates, this.coordinates);    
    arrayCopy(coordinates, this.desiredState);                                                                    //setting our intial desired state to the spawn coordiantes, this can be changed
    
    this.Color = color(random(0, 255), random(0, 255), random(0, 255));
    
    for(int i = 0; i < NUM_AXIS; i++) {lastMeasurments[i] = millis(); integral[i] = 0; forces[i] = 0;}            //initializing our integral terms, and setting forces to 0
    for(int i = 0; i < 3; i++) {constants[i] = 0;}                                                                
    
    agents = (Agent[])append(agents, this);                                                                       //adding this to the list of rigid bodies in the simulations
  }
  
  Agent(int coordinates[], float forces[])
  {
    //use arrayCopy to make deep copies
    arrayCopy(coordinates, this.coordinates);                                                                     
    arrayCopy(forces, this.forces);
    arrayCopy(coordinates, this.desiredState);                                                                    //setting our intial desired state to the spawn coordiantes, this can be changed
    
    this.Color = color(random(0, 255), random(0, 255), random(0, 255));
    
    for(int i = 0; i < NUM_AXIS; i++) {lastMeasurments[i] = millis(); integral[i] = 0;}                           //starting the timer for our measurment clocks and setting our integral sums to zero as PID adjustment hasn't started
    for(int i = 0; i < 3; i++) {constants[i] = 0;}                                                                //if no constants are passed we have to set them to zero
    agents = (Agent[])append(agents, this);                                                                       //adding this to the list of rigid bodies in the simulations
  }
  
  Agent(int coordinates[], float forces[], float constants[], int max[])
  {
    //use arrayCopy for deep copies
    arrayCopy(coordinates, this.coordinates);
    arrayCopy(forces, this.forces);
    arrayCopy(constants, this.constants);
    arrayCopy(coordinates, this.desiredState);
    maxOutputs = new int[3];
    arrayCopy(max, this.maxOutputs);                                                                             //governs the max outputs of each PID field
    this.Color = color(random(0,255), random(0,255), random(0,255));
    
    for(int i = 0; i < NUM_AXIS; i++) {lastMeasurments[i] = millis(); integral[i] = 0;}                          //starting the timer for our measurment clocks and setting our integral sums to zero as PID adjustment hasn't started
    agents = (Agent[])append(agents, this);                                                                      //adding this to the list of rigid bodies in the simulations
  }
  
  Agent(int coordinates[], float forces[], float constants[], int max[], color Color)
  {
    //use arrayCopy for deep copies
    arrayCopy(coordinates, this.coordinates);
    arrayCopy(forces, this.forces);
    arrayCopy(constants, this.constants);
    arrayCopy(coordinates, this.desiredState);
    maxOutputs = new int[3];
    arrayCopy(max, this.maxOutputs);
    this.Color = Color;
    
    for(int i = 0; i < NUM_AXIS; i++) {lastMeasurments[i] = millis(); integral[i] = 0;}                          //starting the timer for our measurment clocks and setting our integral sums to zero as PID adjustment hasn't started
    agents = (Agent[])append(agents, this);                                                                      //adding this to the list of rigid bodies in the simulations
  }
  
  float calcError(int num) {return desiredState[num] - coordinates[num];}                                        //calculates the error on one axis from its desired state to its current state
  
  float calcPIDAdjust(int axis)                                                                                  //runs through PID equations to determine its output to reach the desired state
  {
    
       clock = (millis() - lastMeasurments[axis]) / 1000;                                                        //calculates change in time
       float error = calcError(axis);                                                                            //gets the error
       if(reset == true) {lastErrors[axis] = error; reset = false;}                                              //checks if a new sate was set so we are resetting the reset to avoid falsely perceived error
        
       float proportional = constants[0] * error;
       
       integral[axis] += constants[1] * error * clock;
       
       float derivative = constants[2] * (error - lastErrors[axis]) / clock;
       
       lastErrors[axis] = error;                                                                                //resets our "last" terms to the current error to prep for the next frame
       lastMeasurments[axis] = millis();                                                                        //resets our measurement timer
       
       if(maxOutputs != null)                                                                                   //if maxOutputs are entered then we regulate the output of our PID controller
       {
         if(proportional > maxOutputs[0]) {proportional = maxOutputs[0];}                                       //regulating max output in both positive and negative cases
         else if(abs(proportional) > maxOutputs[0]) {proportional = maxOutputs[0] * -1;}
         
         if(integral[axis] > maxOutputs[1]) {integral[axis] = maxOutputs[1];}                                   //regulating max output in both positive and negative cases
         else if(abs(integral[axis]) > maxOutputs[1]) {integral[axis] = maxOutputs[1] * -1;}
         
         if(derivative > maxOutputs[2]) {derivative = maxOutputs[2];}                                           //regulating max output in both positive and negative cases
         else if(abs(derivative) > maxOutputs[2]) {derivative = maxOutputs[2] * -1;}
       }
       
       float offset = integral[axis] + proportional + derivative;                                               //adding our terms for total output
       return offset;
  
  }
 
 
  void changeConstants(float newConstants[]) {arrayCopy(newConstants, this.constants);}                         //changes PID constants
 
  void applyForce()                                                                                             //calculates the state of the next frame due to forces and PID outputs
  {

      for(int i = 0; i < NUM_AXIS; i++)  //going through each axis
      {
        
          float output = (calcPIDAdjust(i) + forces[i]) * clock;                                                
          
          for(int j = 0; j < agents.length; j++)                                                                //going through all agents in the simulation and checking if they are getting to close
          {
           if(agents[j] == this) {continue;}                                                                    //skip avoiding itself
           output += obstacleAvoidance(i, agents[j]) * clock;                                                   //add in all the avoidance forces from each potential "obstacle"
          }
          
          if(abs(output) > (maxOutputs[0] + maxOutputs[1] - maxOutputs[2]) * clock) {output = (maxOutputs[0] + maxOutputs[1] - maxOutputs[2]) * clock * (output / abs(output));} //derivative is almost always going against the other terms, hence the subtraction
          coordinates[i] += output; //adding that to our state
      
      }
    
      
    //drawing our new ellipse
    fill(Color);
    ellipse(coordinates[0], coordinates[1], 10, 10);
    
  }
 
 
  float calcDistance(int x, int y)                                                                     //returns distance between two points
  {return sqrt((x - coordinates[0])^2 + (y - coordinates[1])^2);}
 
  
   float obstacleAvoidance(int axis, Agent agent2)                                                     //returns an adjustment force to help the drone avoid running into obstacles
   {
     float difference = coordinates[axis] - agent2.coordinates[axis];                                  //how far the drone and object are from eachother, NOTE IT CAN BE NEGATIVE
     for(int i = 0; i < NUM_AXIS; i++) {if(i == axis) {continue;} if(abs(coordinates[i] - agent2.coordinates[i]) > minDistance) {return 0;}} //checking if it is actually lined up on the other axes as well, if not then we know they aren't actually close
     
     if(abs(difference) > maxDistance) {return 0;}                                                     //if they are farther than the maxDistance we start avoiding at, then return no adjustment
     //integral[axis] -= clock * this.constants[1] * difference;
     float totalForce = maxOutputs[0] + maxOutputs[1] - maxOutputs[2];                                 //the max adjust we can use to avoid obstacles
     float mapped = map(abs(difference), maxDistance, minDistance, 0, totalForce);                     //map a scale from the distance we start avoiding at to the min proximity allowed to a scale of zero to the max adjust force
     if(abs(difference) > 0) {mapped *= (difference / abs(difference));}                               //here we preserver the sign of the adjustment, AKA make sure we adjust in the right direction, we check for zero to avoid divide by zero errors

     if(Float.isNaN(mapped))                                                                           //occasionally, due to how the map function works, we get NaN. We just return 0 as usually this means the difference value wasn't far in the range of our map scales I think
     {return 0;}
     else if(abs(mapped) > totalForce)                                                                 //makes sure mapped stays within our bounds and preserves the sign when limiting the output
     {
       mapped = totalForce;
       if(abs(difference) > 0) {mapped *= (difference / abs(difference));}                             //divinding a number by the absolute value of itself is a way of getting the sign of that number
       else if(abs(calcError(axis))> 0) {mapped *= abs(calcError(axis)) / (calcError(axis));} 
     }
     
     return mapped;
   } 
  
}