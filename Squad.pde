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
  
  //int[] formationFunction()
  //{
  //  int adjusts[] = {0, 0};
  //  try
  //  {
  //    int mag = ((members.length / 8) * 20) + 20;          
  //    float angle = map((members.length) % 8, 0, 8, 0, 2 * PI);
  //    adjusts[0] = int(cos(angle) * mag);
  //    adjusts[1] = int(sin(angle) * mag);
  //    println("Members Length: ", members.length);
  //    println("Mag: ", mag);
  //    println("Angle: ", angle);
  //    println("X: ", adjusts[0]);
  //    println("Y: ", adjusts[1]);
  //    println();
  //    return adjusts;
  //  }
  //  catch(NullPointerException e)
  //  {
  //    int mag = ((0 / 8) * 20) + 20;
  //    float angle = map(0, 0, 8, 0, 2 * PI);
  //    adjusts[0] = int(cos(angle) * mag);
  //    adjusts[1] = int(sin(angle) * mag);
  //    println("Num Members: ",0);
  //    println("Mag: ", mag);
  //    println("Angle: ", angle);
  //    println("X: ", adjusts[0]);
  //    println("Y: ", adjusts[1]);
  //    println();
  //    return adjusts;
  //  }
    
//  }
  
  int []formationFunction()
 {

  int adjusts[] = {0, 0};
  try
  {
    mag = shellNum * 20 + 20;
    float angle = map((members.length) % shellMax, 0, shellMax, 0, 2 * PI);  
    adjusts[0] = int(cos(angle) * mag);
    adjusts[1] = int(sin(angle) * mag);
    numInShell++;
    if((shellNum == 0 && (numInShell == shellMax - 1)) || numInShell == shellMax) {numInShell = 0; shellMax = int(shellMax * (float(mag + 20) / mag)); shellNum++;}
    return adjusts;
  }
  catch(NullPointerException e)
  {
          adjusts[0] = 20;
          adjusts[1] = 0;    
          numInShell++;
          return adjusts;
  }

}
  
  void createSquadMate(int num) //takes in how many new drones you want to spawn
  {
    for(int i = 0; i < num; i++) {createSquadMate();}
  }
  
  void createSquadMate()
  {
    Drone newDrone;
    Agent tempAgent; //will be the rigid body for our newDrone
    try
    {
      int tempCoordinates[] = {squadLeader.droneBody.coordinates[0] + (members.length + 1) * 10, squadLeader.droneBody.coordinates[1] + (members.length + 1) * 10};
      tempAgent = new Agent(tempCoordinates, squadLeader.droneBody.forces, squadLeader.droneBody.constants);  
      int tempAdjust[] = formationFunction();
      newDrone = new Drone(tempAgent, tempAdjust);
    }
    catch (NullPointerException e)
    {
      int tempCoordinates[] = {squadLeader.droneBody.coordinates[0] + 10, squadLeader.droneBody.coordinates[1] + 10};
      tempAgent = new Agent(tempCoordinates, squadLeader.droneBody.forces, squadLeader.droneBody.constants);
      int tempAdjust[] = formationFunction();
      newDrone = new Drone(tempAgent, tempAdjust);
    }
    addSquadMate(newDrone);
  }
  
  void addSquadMate(Drone newDrone)        //actually adds the new drone to the squad
  {
    if(members == null) {members = new Drone[1]; members[0] = newDrone;}
    else{members = (Drone[])append(members, newDrone);}
    members[members.length - 1].lead = squadLeader;
    print("numInShell: ", numInShell);
    print();
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
    Drone tempDrone = members[members.length - 1];
    members = (Drone[])shorten(members);
    numInShell--;
    println("Num In Shell: ", numInShell);
    println("Shell Num: ", shellNum);
    println("Shell Max: ", shellMax);
    println();
    if(numInShell == -1)
    {
      shellNum--;
      shellMax = int(shellMax * float((mag - 20) / mag));
      numInShell = shellMax;      
    }
    return tempDrone;  
  }
  
  void update()
  {
    squadLeader.update();
    if(members != null) {for(int i = 0; i < members.length; i++) {members[i].update();}}
  }
  
  void move(int x, int y) {squadLeader.move(x, y);}
  
};