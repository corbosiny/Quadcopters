class Drone
{
  
  Agent droneBody;
  int adjusts[];
  int tolerance;
  Drone lead = null;
  
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
   
  void update()
  {
    if(lead != null)
    {
      if(droneBody.desiredState[0] != lead.droneBody.coordinates[0] || droneBody.desiredState[1] != lead.droneBody.coordinates[1])
      {
        droneBody.desiredState[0] = lead.droneBody.coordinates[0] + adjusts[0];
        droneBody.desiredState[1] = lead.droneBody.coordinates[1] + adjusts[1];
        droneBody.reset = true;
      }
      
    }
    
    droneBody.applyForce();   
    
  }
   
}