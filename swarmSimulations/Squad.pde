class Squad
{
  
  TestDrone squadLeader;
  Drone members[];
  
  Squad(TestDrone leader)
  {
    squadLeader = leader;
    members = null;
  }
  
  Squad(TestDrone leader, Drone members[])
  {
    squadLeader = leader;
    this.members = members;
    for(int i = 0; i < members.length; i++) {this.members[i].lead = squadLeader;}
  }

  Squad(TestDrone leader, int numMembers)
  {
   squadLeader = leader;
   members = new Drone[numMembers];
   for(int i = 0; i < numMembers; i++) 
   {
       int tempAdjust[] = {10 + i * 10, 10 + i * 10}; 
       Agent tempAgent = new Agent(squadLeader.droneBody.coordinates, squadLeader.droneBody.forces, squadLeader.droneBody.constants);
       members[i] = new Drone(tempAgent, tempAdjust);
       members[i].lead = squadLeader;
   }
  
  }
  
  void createSquadMate(int num)
  {
    for(int i = 0; i < num; i++) {createSquadMate();}
  }
  
  void createSquadMate()
  {
    Drone newDrone;
    Agent tempAgent;
    try
    {
      int tempCoordinates[] = {squadLeader.droneBody.coordinates[0] + (members.length + 1) * 10, squadLeader.droneBody.coordinates[1] + (members.length + 1) * 10};
      int tempAdjust[] = {10 + (members.length - 1) * 10, 10 + (members.length - 1) * 10};
    }
    catch (NullPointerException e)
    {
      int tempCoordinates[] = {squadLeader.droneBody.coordinates[0] + 10, squadLeader.droneBody.coordinates[1] + 10};
      tempAgent = new Agent(tempCoordinates, squadLeader.droneBody.forces, squadLeader.droneBody.constants);
      int tempAdjust[] = {10, 10};
    }
    newDrone = new Drone(tempAgent, tempAdjust);
    addSquadMate(newDrone);
  }
  
  void addSquadMate(Drone newDrone)
  {
    if(members == null) {members = new Drone[1]; members[0] = newDrone;}
    else{members = (Drone[])append(members, newDrone);}
    members[members.length - 1].lead = squadLeader;
  }
  
  void update()
  {
    squadLeader.update();
    if(members != null) {for(int i = 0; i < members.length; i++) {members[i].update();}}
  }
  
  void move(int x, int y) {squadLeader.move(x, y);}
  
};