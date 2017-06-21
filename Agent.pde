class Agent    //Essentially a rigid body object of the Drone
{
  
  static final int NUM_AXIS = 2;
  int maxOutputs[] = new int[3]; //limits speed the agents can go
  int coordinates[];                  //current state of the agent
  float forces[] = new float[NUM_AXIS];             //holds the forces in each direction
  float constants[] = new float[NUM_AXIS];                  //PID constants
  float integral[] = new float[NUM_AXIS];                 //holds our running integral calculationg
  float derivative = 0;
  float lastErrors[] = {0, 0};        //holds the previous error term for derivatives
  boolean reset = false;              //lets us know when a new state is set so we can canel out derivative spikes
  int desiredState[] = {0, 0};        //the state the agent is trying to achieve
  float lastMeasurments[] = {0, 0};   //last time we took measurments at used for time based calculations like derivative and integral
  float clock;                        //holds change of time calculations converted into seconds
  color Color;                        //color of the dronew
  
  Agent(int coordinates[])
  {
                 
    this.coordinates = coordinates;               
    this.desiredState[0] = this.coordinates[0];
    this.desiredState[1] = this.coordinates[1];
    this.lastMeasurments[0] = millis();
    this.lastMeasurments[1] = millis();
    
    this.Color = color(random(0, 255), random(0, 255), random(0, 255));
    
    for(int i = 0; i < NUM_AXIS; i++) {integral[i] = 0; forces[i] = 0;}
    for(int i = 0; i < 3; i++) {constants[i] = 0;}
    agents = (Agent[])append(agents, this);
  }
  
  Agent(int coordinates[], float forces[])
  {
    this.coordinates = coordinates;
    this.forces = forces;
    this.desiredState[0] = this.coordinates[0];
    this.desiredState[1] = this.coordinates[1];
    this.lastMeasurments[0] = millis();
    this.lastMeasurments[1] = millis();
    
    this.Color = color(random(0, 255), random(0, 255), random(0, 255));
    
    for(int i = 0; i < NUM_AXIS; i++) {integral[i] = 0;}
    for(int i = 0; i < 3; i++) {constants[i] = 0;}
    agents = (Agent[])append(agents, this);
  }
  
  Agent(int coordinates[], float forces[], float constants[], int max[])
  {
    this.coordinates = coordinates;
    this.forces = forces;
    this.constants = constants;
    this.desiredState[0] = this.coordinates[0];
    this.desiredState[1] = this.coordinates[1];
    this.lastMeasurments[0] = millis();
    this.lastMeasurments[1] = millis();
    this.maxOutputs = max;
    this.Color = color(random(0,255), random(0,255), random(0,255));
    
    for(int i = 0; i < NUM_AXIS; i++) {integral[i] = 0;}
    agents = (Agent[])append(agents, this);
  }
  
  Agent(int coordinates[], float forces[], float constants[], int max[], color Color)
  {
    
    this.coordinates = coordinates;
    this.forces = forces;
    this.constants = constants;
    this.desiredState[0] = this.coordinates[0] + (int) random(0, 50);
    this.desiredState[1] = this.coordinates[1] + (int) random(0, 50);
    this.lastMeasurments[0] = millis();
    this.lastMeasurments[1] = millis();
    this.maxOutputs = max;
    this.Color = Color;
    
    for(int i = 0; i < NUM_AXIS; i++) {integral[i] = 0;}
    agents = (Agent[])append(agents, this);
  }
  
  float calcError(int num) {return this.desiredState[num] - this.coordinates[num];}  //calculates the error on one axis from its desired state to its current state
  
  float calcPIDAdjust(int axis) //runs through PID equations to determine its output to reach the desired state
  {
    
       this.clock = (millis() - this.lastMeasurments[axis]) / 1000;  //calculates change in time
       float error = this.calcError(axis);                           //gets the error
       if(this.reset == true) {lastErrors[axis] = error; this.reset = false;}  //checks if a new sate was set so we are resetting the reset to avoid falsely perceived error
        
       float proportional = this.constants[0] * error;
       
       this.integral[axis] += this.constants[1] * error * clock;
       
       this.derivative = this.constants[2] * (error - lastErrors[axis]) / clock;
       
       this.lastErrors[axis] = error;      //resets our "last" terms to the current error to prep for the next frame
       this.lastMeasurments[axis] = millis();
       
       if(this.maxOutputs != null)
       {
         if(proportional > this.maxOutputs[0]) {proportional = this.maxOutputs[0];}    //regulating max output in both positive and negative cases
         else if(abs(proportional) > this.maxOutputs[0]) {proportional = this.maxOutputs[0] * -1;}
         
         if(this.integral[axis] > this.maxOutputs[1]) {this.integral[axis] = this.maxOutputs[1];} //regulating max output in both positive and negative cases
         else if(abs(this.integral[axis]) > this.maxOutputs[1]) {this.integral[axis] = this.maxOutputs[1] * -1;}
         
         if(this.derivative > this.maxOutputs[2]) {this.derivative = this.maxOutputs[2];} //regulating max output in both positive and negative cases
         else if(abs(this.derivative) > this.maxOutputs[2]) {this.derivative = this.maxOutputs[2] * -1;}
       }
       
       float offset = this.integral[axis] + proportional + derivative;  //adding our terms for total output
       return offset;
  
  }
 
 
  void changeConstants(float newConstants[]) {this.constants = newConstants;} //changes PID constants
 
  void applyForce()    //calculates the state of the next frame due to forces and PID outputs
  {

      for(int i = 0; i < NUM_AXIS; i++)  //going through each axis
      {
          float PIDoutput = calcPIDAdjust(i);
          float finalForce = (this.forces[i]); //calculating the force acting on the agent
         
          float output = (PIDoutput + finalForce) * this.clock; //final state change is the sum of the force and the PID outputs 
          
          for(int j = 0; j < agents.length; j++)
          {
           if(agents[j] == this) {continue;}
           output += obstacleAvoidance(i, agents[j]) * this.clock;
          }
          
          if(abs(output) > (maxOutputs[0] + maxOutputs[1] - maxOutputs[2]) * this.clock) {output = (maxOutputs[0] + maxOutputs[1] - maxOutputs[2]) * this.clock * (output / abs(output));}
          this.coordinates[i] += output; //adding that to our state
      
      }
    
      
    
    fill(this.Color); //drawing our new ellipse
    ellipse(this.coordinates[0], this.coordinates[1], 10, 10);
    
  }
 
 
  float calcDistance(int x, int y) //returns distance between two points
  {return sqrt((x - coordinates[0])^2 + (y - coordinates[1])^2);}
 
  
   float obstacleAvoidance(int axis, Agent agent2)
   {
     float difference = this.coordinates[axis] - agent2.coordinates[axis];
     for(int i = 0; i < NUM_AXIS; i++) {if(i == axis) {continue;} if(abs(this.coordinates[i] - agent2.coordinates[i]) > minDistance) {return 0;}}
     
     if(abs(difference) > maxDistance) {return 0;}
     //this.integral[axis] -= this.clock * this.constants[1] * difference;
     float currentForce = calcError(axis) * this.constants[0] + this.integral[axis] + this.derivative;
     float totalForce = maxOutputs[0] + maxOutputs[1] - maxOutputs[2];
     float mapped = map(abs(difference), maxDistance, minDistance, 0, currentForce);
     if(abs(difference) > 0) {mapped *= (difference / abs(difference));}

     if(Float.isNaN(mapped) || abs(mapped) > totalForce) 
     {
       mapped = totalForce;
       if(abs(difference) > 0) {mapped *= (difference / abs(difference));}
       else if(abs(calcError(axis))> 0) {mapped *= abs(calcError(axis)) / (calcError(axis));} 
     }
     return mapped;
   } 
  
}