class Drone
{
  
  static final int NUM_AXIS = 2;                          //number of dimensions in the simulation
  Agent droneBody;                                        //the rigid body component of our drone, handles all the physics and PID controls 
  int adjusts[] = new int[NUM_AXIS];                      //how its coordinates should be adjusted based off the leaders position, so 5,0 means he stays 5 pixels to the right of the leader
  Squad squad = null;                                     //its squad leader, if this isn't null it will try and follow this leader
  boolean isLeader = false;                               
  
  Drone(Agent newBody)
  {
   this.droneBody = newBody;                       
   this.squad= null;  
  }
  
  Drone(Agent newBody, int adjusts[])                                           //the regular drone alone wont do anything until given a squad
  {
    this.droneBody = newBody;
    arrayCopy(adjusts, this.adjusts);          
  }
  
  Drone(Agent newBody, int adjusts[], Squad squad)                              //when given a squad and adjusts the drone will atempt to follow the squad leader based on the given adjusts
  {
    this.droneBody = newBody;
    arrayCopy(adjusts, this.adjusts);    
    this.squad = squad;
  }

  Drone(Agent newBody, Squad squad)                                             //if just given a body and a squad we assume this drone is the leader
  {
    this.droneBody = newBody;
    arrayCopy(adjusts, this.adjusts);    
    this.squad = squad;
    this.isLeader = true;
  }
 
  void update() 
  {
    if(squad != null && !isLeader)                                              //if it has a leader the drone will try to update it's desired state to follow the leader and stay in formation
    {
      for(int j = 0; j < NUM_AXIS; j++)
      {
         if(droneBody.desiredState[j] != squad.squadLeader.droneBody.desiredState[j] + adjusts[j]) {droneBody.desiredState[j] = squad.squadLeader.droneBody.desiredState[j] + adjusts[j]; droneBody.reset = true;}
      }
    }
    droneBody.applyForce();   
  }
   
};