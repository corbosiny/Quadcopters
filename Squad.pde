class Squad
{
  
  TestDrone squadLeader;
  Drone members[];

  int shellMax = 8;
  int numInShell = 0;
  int shellNum = 0;
  int mag;  
  
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
  
  int []formationFunction()
 {

        int adjusts[] = {0, 0};
        mag = shellNum * 20 + 20;
        float angle = map(numInShell, 0, shellMax, 0, 2 * PI);
        adjusts[0] = int(cos(angle) * mag);
        adjusts[1] = int(sin(angle) * mag);
        numInShell++;
        return adjusts;

 }
 
 int []formationFunction(int numIn, int shellNum, int shellMax)
 {
   int adjusts[] = {0,0};
   
   return adjusts;
 }
 
 void reverseFormationFunction()
 {
    numInShell--;
    if(numInShell == -1 && shellNum > 1)
    {
      shellNum--;
      shellMax = int(shellMax * (float(mag - 20) / float(mag)));
      numInShell = shellMax - 1;  
      mag = shellNum * 20 + 20;
    }
    else if(numInShell == -1 && shellNum == 1 )
    {
     numInShell = 7;
     shellNum = 0;
     shellMax = 8;
     mag = 20;
    }
    
 }
  
  Drone[] createSquadMate(int num) //takes in how many new drones you want to spawn
  {
    Drone drones[] = new Drone[num];
    for(int i = 0; i < num; i++) {drones[num] = createSquadMate();}
    return drones;
  } 
  
  Drone createSquadMate()
  {
    Drone newDrone;
    Agent tempAgent; //will be the rigid body for our newDrone
    try
    {
      int tempCoordinates[] = {squadLeader.droneBody.coordinates[0] + (shellNum + 1) * 20 + 20, squadLeader.droneBody.coordinates[1] + (shellNum + 1) * 20};
      tempAgent = new Agent(tempCoordinates, squadLeader.droneBody.forces, squadLeader.droneBody.constants);  
      newDrone = new Drone(tempAgent);
    }
    catch (NullPointerException e)
    {
      int tempCoordinates[] = {squadLeader.droneBody.coordinates[0] + 20, squadLeader.droneBody.coordinates[1]};
      tempAgent = new Agent(tempCoordinates, squadLeader.droneBody.forces, squadLeader.droneBody.constants);
      newDrone = new Drone(tempAgent);
    }
    return newDrone;
  }
  
  void addSquadMate(Drone newDrone)        //actually adds the new drone to the squad
  {
    if(members == null) {members = new Drone[1]; members[0] = newDrone;}
    else{members = (Drone[])append(members, newDrone);}
    int tempAdjust[] = formationFunction();
    newDrone.adjusts = tempAdjust;
    spaceOut();
    if(numInShell == shellMax) {numInShell = 0; shellMax = int(shellMax * (float(mag + 20) / mag)); shellNum++;}
    members[members.length - 1].lead = squadLeader;
  }
  
  void addSquadMate(Drone newDrone, int[] adjusts)        //actually adds the new drone to the squad
  {
    if(members == null) {members = new Drone[1]; members[0] = newDrone;}
    else{members = (Drone[])append(members, newDrone);}
    members[members.length - 1].lead = squadLeader;
    members[members.length - 1].adjusts = adjusts;
  }
  
  Drone removeSquadMate()
  {
    Drone tempDrone = null;
    if(members == null || members.length == 0) {return tempDrone;}
    tempDrone = members[members.length - 1];
    members = (Drone[])shorten(members);
    reverseFormationFunction();
    spaceOut();
    return tempDrone;  
  }
 
 int transferDrone(Squad squad2) //transfers a drone from squad1 to squad 2
 {
  if(this.members == null || this.members.length == 0) {return 0;}
  Drone tempDrone = this.removeSquadMate();
  squad2.addSquadMate(tempDrone);
  return 1;
 }
  
  void update()
  {
    squadLeader.update();
    if(members != null) {for(int i = 0; i < members.length; i++) {members[i].update();}}
  }
  
 void move(int x, int y) {squadLeader.move(x, y);}
  
 void spaceOut()
 {
    for(int i = members.length - 1; i > members.length - 1 - numInShell; i--)
    {
      mag = shellNum * 20 + 20;
      float angle = map(i - members.length + numInShell, 0, numInShell, 0, 2 * PI);
      int tempAdjusts[] = {0,0};
      tempAdjusts[0] = int(mag * cos(angle));
      tempAdjusts[1] = int(mag * sin(angle));
      this.members[i].adjusts = tempAdjusts;
    }
 }
  
 void squadDebug(char[] startTitle)
 {
   println(startTitle);
   println("Shell Num: ", shellNum);
   println("Num In Shell: ", numInShell);
   println("Shell Max: ", shellMax);
   println("\n\n");
 }
  
};