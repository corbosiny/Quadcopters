class Drone
{
  
  static final int NUM_AXIS = 2;                          
  RigidBody droneBody;                                   
  int coordinatesAdjusts[] = new int[NUM_AXIS];           //how its coordinates should be adjusted based off the leaders position, so 5,0 means he stays 5 pixels to the right of the leader
  Squad squad = null;                                     //its squad leader, if this isn't null it will try and follow this leader
  boolean isLeader = false;                               
  
  Drone(RigidBody newBody)
  {
   this.droneBody = newBody;                       
   this.squad= null;  
  }
  
  Drone(RigidBody newBody, int coordinatesAdjusts[])                                           //the regular drone alone wont do anything until given a squad
  {
    this.droneBody = newBody;
    arrayCopy(coordinatesAdjusts, this.coordinatesAdjusts);          
  }
  
  Drone(RigidBody newBody, int coordinatesAdjusts[], Squad squad)                              //when given a squad and coordinatesAdjusts the drone will atempt to follow the squad leader based on the given coordinatesAdjusts
  {
    this.droneBody = newBody;
    arrayCopy(coordinatesAdjusts, this.coordinatesAdjusts);    
    this.squad = squad;
  }

  Drone(RigidBody newBody, Squad squad)                                             //if just given a body and a squad we assume this drone is the leader
  {
    this.droneBody = newBody;
    arrayCopy(coordinatesAdjusts, this.coordinatesAdjusts);    
    this.squad = squad;
    this.isLeader = true;
  }
 
  void updateCoordinates() 
  {
    if(squad != null && !isLeader)                                              //if it has a leader the drone will try to update it's desired state to follow the leader and stay in formation
    {
      for(int j = 0; j < NUM_AXIS; j++)
      {
         if(droneBody.desiredState[j] != squad.squadLeader.droneBody.desiredState[j] + coordinatesAdjusts[j]) {droneBody.desiredState[j] = squad.squadLeader.droneBody.desiredState[j] + coordinatesAdjusts[j]; droneBody.desiredStateReset = true;}
      }
    }
    droneBody.applyForces();   
  }
   
};