class Agent
{
  
  int maxOutputs[] = {150, 150, 150}; //limits speed the agents can go
  int coordinates[];                  //current state of the agent
  float forces[] = {0,0};             //holds the forces in each direction
  float constants[];                  //PID constants
  float integral = 0;                 //holds our running integral calculationg
  float lastErrors[] = {0, 0};        //holds the previous error term for derivatives
  boolean reset = false;              //lets us know when a new state is set so we can canel out derivative spikes
  int desiredState[] = {0, 0};        //the state the agent is trying to achieve
  float lastMeasurments[] = {0, 0};   //last time we took measurments at used for time based calculations like derivative and integral
  float clock;                        //holds change of time calculations converted into seconds
  color Color;                        //color of the drone
  
  Agent(int coordinates[])
  {
  
    float tempForces[] = {0,0,0};
    float constants[] = {0,0,0};
    this.coordinates = coordinates;
    this.forces = tempForces;
    this.constants = constants;
    this.desiredState[0] = this.coordinates[0] + (int) random(0, 50);
    this.desiredState[1] = this.coordinates[1] + (int) random(0, 50);
    this.lastMeasurments[0] = millis();
    this.lastMeasurments[1] = millis();
    
    this.Color = color(random(0, 255), random(0, 255), random(0, 255));
    
  }
  
  Agent(int coordinates[], float forces[])
  {
    float constants[] = {0,0,0};
    this.coordinates = coordinates;
    this.forces = forces;
    this.constants = constants;
    this.desiredState[0] = this.coordinates[0] + (int) random(0, 50);
    this.desiredState[1] = this.coordinates[1] + (int) random(0, 50);
    this.lastMeasurments[0] = millis();
    this.lastMeasurments[1] = millis();
    
    this.Color = color(random(0, 255), random(0, 255), random(0, 255));
  }
  
  Agent(int coordinates[], float forces[], float constants[])
  {
    this.coordinates = coordinates;
    this.forces = forces;
    this.constants = constants;
    this.desiredState[0] = this.coordinates[0] + 100;
    this.desiredState[1] = this.coordinates[1] + 30;
    this.lastMeasurments[0] = millis();
    this.lastMeasurments[1] = millis();
    
    this.Color = color(random(0,255), random(0,255), random(0,255));
  }
  
  Agent(int coordinates[], float forces[], float constants[], color Color)
  {
    
    this.coordinates = coordinates;
    this.forces = forces;
    this.constants = constants;
    this.desiredState[0] = this.coordinates[0] + (int) random(0, 50);
    this.desiredState[1] = this.coordinates[1] + (int) random(0, 50);
    this.lastMeasurments[0] = millis();
    this.lastMeasurments[1] = millis();
    
    this.Color = Color;
    
  }
  
  float calcError(int num) {return this.desiredState[num] - this.coordinates[num];}  //calculates the error on one axis from its desired state to its current state
  
  float calcPIDAdjust(int axis) //runs through PID equations to determine its output to reach the desired state
  {
    
       this.clock = (millis() - this.lastMeasurments[axis]) / 1000;  //calculates change in time
       float error = this.calcError(axis);                           //gets the error
       if(this.reset == true) {lastErrors[axis] = error; this.reset = false;}  //checks if a new sate was set
        
       float proportional = this.constants[0] * error;
       if(proportional > this.maxOutputs[0]) {proportional = this.maxOutputs[0];}    //regulating max output in both positive and negative cases
       else if(abs(proportional) > this.maxOutputs[0]) {proportional = this.maxOutputs[0] * -1;}
       
       this.integral += this.constants[1] * error * clock;
       if(this.integral > this.maxOutputs[1]) {this.integral = this.maxOutputs[1];} //regulating max output in both positive and negative cases
       else if(abs(this.integral) > this.maxOutputs[1]) {this.integral = this.maxOutputs[1] * -1;}
       
       float derivative = this.constants[2] * (error - lastErrors[axis]) / clock;
       if(derivative > this.maxOutputs[2]) {derivative = this.maxOutputs[2];} //regulating max output in both positive and negative cases
       else if(abs(derivative) > this.maxOutputs[2]) {derivative = this.maxOutputs[2] * -1;}
       
       this.lastErrors[axis] = error;      //resets our "last" terms to the current error to prep for the next frame
       this.lastMeasurments[axis] = millis();
       
       float offset = this.integral + proportional + derivative;  //adding our terms for total output
       return offset;
  
  }
 
 
  void changeConstants(float newConstants[]) {this.constants = newConstants;} //changes PID constants
 
  void applyForce()    //calculates the state of the next frame due to forces and PID outputs
  {

      for(int i = 0; i < 2; i++)  //going through each axis
      {
          float PIDoutput = calcPIDAdjust(i);
          float finalForce = (this.forces[i]); //calculating the force acting on the agent
         
          float output = (PIDoutput + finalForce) * this.clock; //final state change is the sum of the force and the PID outputs
          this.coordinates[i] += output; //adding that to our state
      }
    
    fill(this.Color); //drawing our new ellipse
    ellipse(this.coordinates[0], this.coordinates[1], 10, 10);
    
  }
 
 
  float calcDistance(int x, int y)
  {return sqrt((x - coordinates[0])^2 + (y - coordinates[1])^2);}
  
  
}