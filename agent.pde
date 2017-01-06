class Agent
{
  
  int coordinates[];                                      
  float force;                                //will hold the force acting on the object, used to trigger PID response
  float constants[];                          //the constants for our PID equations, 0 = proportional, 1 = integral, 2 = derivative
  float integral = 0;                         //stores the agents integral tracking
  float lastError = 0;                        //used for tracking the derivative term
  boolean reset = false;                      //used in case the targets are changed to avoid a seeming spike in the derivative or integral terms
  int desiredState[] = {0, 0};                
  float lastMeasurment;                       //in milliseconds
  float clock;
  
  Agent(int coordinates[], float force, float constants[])  //needs to know its initial coordinates, the magnitude of the force acting on it, then the PID constants
  {
    
    this.coordinates = coordinates;
    this.force = force;
    this.constants = constants;
    this.desiredState[0] = coordinates[0] + 0;
    this.desiredState[1] = coordinates[1] + 0;
    this.lastMeasurment = millis();
    
  }
  
  float calcError() {return this.desiredState[1] - this.coordinates[1];}  //the erros is used in the PID equations
  
  float calcPIDAdjust()           //runs through the PID equations and calculates the correction output
  {
    
   this.clock = (millis() - this.lastMeasurment) / 1000;    //time step is used for the integral and derivative equations
   float error = this.calcError();
   print(clock);
   print(" ");
   print(error);
   print(" ");
   if(this.reset == true) {lastError = error; reset = false;}   //is only set when the desired state is changed to avoid spikes in the derivative and integral terms
    
   float proportional = this.constants[0] * error;
   this.integral += this.constants[1] * error * clock;
   float derivative = this.constants[2] * (error - lastError) / clock;
   print(proportional);
   print(" ");
   print(this.integral);
   print(" ");
   print(derivative);
   print(" ");
   this.lastError = error;
   float offset = this.integral + proportional + derivative;    //sums all outputs to get the total correction force for our PID controller
   this.lastMeasurment = millis();
   return offset;
  
  }
 
 void changeState(int newDesiredState[])                        //used to set a new desired state for the agent, in x, y coordinates
 {
   
    this.desiredState = newDesiredState;
    integral = 0;
    reset = true;
    
 }
 
  void changeConstants(float newConstants[])                  //constants are set up like: proportional, integral, derivative
  {
  
    this.constants = newConstants;
    
  }
 
  void applyForce()   //here is where the correction force is updated and the state of the agent is updated
  {
   
    float PIDoutput = calcPIDAdjust();          
    float output = (PIDoutput + this.force * sin(millis)) * this.clock;       //play with the equation for the acting force to see how the PIDs respond, ex: sinosidal force, constant force, random, etc.
    print(sin(millis()));
    print(" ");
    print(PIDoutput);
    print("\n");
    this.coordinates[1] += output; //updates coordinates based off of resultant force on the agent
    fill(255);
    ellipse(this.coordinates[0], this.coordinates[1], 10, 10); //draws the marker representing the agent here
    
  }
  
}
