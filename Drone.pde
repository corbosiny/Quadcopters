class Drone
{
  
  Agent droneBody;     //the rigid body component of our drone, handles all the physics and PID controls 
  int adjusts[];       //how its coordinates should be adjusted based off the leaders position, so 5,0 means he stays 5 pixels to the right of the leader
  int tolerance;       //the range it can be to its target coordinates where it will stop adjusting
  Drone lead = null;   //its squad leader, if this isn't null it will try and follow this leader
  
  Drone(Agent newBody)
  {
   this.droneBody = newBody; 
   this.lead = null;  
  }
  
  Drone(Agent newBody, int adjusts[]) 
  {
    this.droneBody = newBody;
    this.adjusts = adjusts;
    this.tolerance = 0;
  }
  
  Drone(Agent newBody, int adjusts[] , int tolerance) 
  {
    this.droneBody = newBody;
    this.adjusts = adjusts;
    this.tolerance = tolerance;
  }
  
   
  void update() //applies
  {
    if(lead != null)
    {
      if(droneBody.desiredState[0] != lead.droneBody.desiredState[0] + adjusts[0] || droneBody.desiredState[1] != lead.droneBody.desiredState[1] + adjusts[1])
      {
        droneBody.desiredState[0] = lead.droneBody.desiredState[0] + adjusts[0];
        droneBody.desiredState[1] = lead.droneBody.desiredState[1] + adjusts[1];
        droneBody.reset = true;
      }
      
    }
    
    droneBody.applyForce();   
    
  }
   
}