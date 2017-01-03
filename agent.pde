class Agent
{
  
  int coordinates[];
  float force;
  float constants[];
  float integral = 0;
  float lastError = 0;
  boolean reset = false;
  int desiredState[] = {0, 0};
  float lastMeasurment;
  float clock;
  
  Agent(int coordinates[], float force, float constants[])
  {
    
    this.coordinates = coordinates;
    this.force = force;
    this.constants = constants;
    this.desiredState[0] = coordinates[0] + 0;
    this.desiredState[1] = coordinates[1] + 0;
    this.lastMeasurment = millis();
    
  }
  
  float calcError() {return this.desiredState[1] - this.coordinates[1];}
  
  float calcPIDAdjust()
  {
    
   this.clock = (millis() - this.lastMeasurment) / 1000;
   float error = this.calcError();
   print(clock);
   print(" ");
   print(error);
   print(" ");
   if(this.reset == true) {lastError = error; reset = false;}
    
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
   float offset = this.integral + proportional + derivative;
   this.lastMeasurment = millis();
   return offset;
  
  }
 
 void changeState(int newDesiredState[])
 {
   
    this.desiredState = newDesiredState;
    integral = 0;
    reset = true;
    
 }
 
  void changeConstants(float newConstants[])
  {
  
    this.constants = newConstants;
    
  }
 
  void applyForce()
  {
   
    float PIDoutput = calcPIDAdjust();
    float output = (PIDoutput + this.force * sin(millis)) * this.clock;
    print(sin(millis()));
    print(" ");
    print(PIDoutput);
    print("\n");
    this.coordinates[1] += output;
    fill(255);
    ellipse(this.coordinates[0], this.coordinates[1], 10, 10);
    
  }
  
}