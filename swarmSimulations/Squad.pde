class Squad
{
  
  static final int NUM_AXIS = 2;                                           
  
  Drone squadLeader;                                                    
  Drone members[];                                                          

  //these will not be in here in the drone code,
  //java does not support static variables in methods so,
  //I have them declared as class members, but in the drone they will belong
  //only to the specific formation function
  int originalMax = 6;
  int maxDronesInShell = originalMax;
  int numDronesInShell = 0;
  int shellNum = 0;
  int shellRadius = maxDistance + 5;

  Squad(Drone squadLeader)                                                          //all a squad needs is a leader to be formed
  {
    this.squadLeader = squadLeader;
    squadLeader.isLeader = true;
    members = null;
    squads = (Squad[])append(squads, this);
  }
  
  Squad(Drone squadLeader, Drone members[])                                        //a squad can be formed with a leader and a list of members as well
  {
    this.squadLeader = squadLeader;
    squadLeader.isLeader = true;
    arrayCopy(members, this.members);
    for(int i = 0; i < members.length; i++) {this.members[i].squad = this;}
    squads = (Squad[])append(squads, this);
  }

  Squad(Drone squadLeader, int numMembers)                                        //instead of members pre-made, you can specify a number to make and new drones will be created
  {
   this.squadLeader = squadLeader;
   squadLeader.isLeader = true;
   members = new Drone[numMembers];
   for(int i = 0; i < numMembers; i++) 
   {
       int tempAdjust[] = calcFormationPosition(); 
       RigidBody tempRigidBody = new RigidBody(squadLeader.droneBody.coordinates, squadLeader.droneBody.outsideForces, squadLeader.droneBody.PIDconstants, squadLeader.droneBody.maxPIDoutputs);
       members[i] = new Drone(tempRigidBody, tempAdjust);
       members[i].squad = this;
   }
   squads = (Squad[])append(squads, this);
  }
  
 //for all formation functions the flight systems manual should be read to understand its role and how to design a new one 
 int []calcFormationPosition()    //self contained, this can be replaced with other functions and only needs to return the adjusts of a new drone
 {
        numDronesInShell++;
        spaceOutDronesInOuterShell();
        float mag = shellNum * shellRadius + shellRadius;
        if(numDronesInShell == maxDronesInShell) {numDronesInShell = 0; mag = shellNum * shellRadius + shellRadius; maxDronesInShell = int(maxDronesInShell * ((mag + shellRadius) / mag)); shellNum++;}
        return members[members.length - 1].coordinatesAdjusts;
 }
  
 //does the exact opposite of the formation adjusts function, it's main job is to update the variables keeping track of where the function formula is at due to drones leaving the squad
 //for our purpose it also recalculates outer shell spacing when a drone leaves the squad
 void reverseFormationFunction()
 {
    numDronesInShell--;
    if(numDronesInShell == -1)
    {
      float mag = shellNum * shellRadius + shellRadius;
      shellNum--;
      maxDronesInShell = int(maxDronesInShell * ((mag - shellRadius) / (mag)));
      numDronesInShell = maxDronesInShell - 1;  
    }
    spaceOutDronesInOuterShell();
 }
  
  Drone[] createSquadMate(int num) //takes in how many new drones you want to spawn, returns a list of the created drones
  {
    Drone drones[] = new Drone[num];
    for(int i = 0; i < num; i++) {drones[num] = createSquadMate();}
    return drones;
  } 
  
  Drone createSquadMate()
  {
    Drone newDrone;
    RigidBody tempRigidBody;                                       //will be the rigid body for our newDrone
    int tempCoordinates[] = new int[NUM_AXIS];
    for(int i = 0; i < NUM_AXIS; i++) {tempCoordinates[i] = squadLeader.droneBody.coordinates[i] + (shellNum + 2) * shellRadius + shellRadius;}       //making up some spawn coordinates, spawns the drone just outside the current formation no matter the size
    tempRigidBody = new RigidBody(tempCoordinates, squadLeader.droneBody.outsideForces, squadLeader.droneBody.PIDconstants, squadLeader.droneBody.maxPIDoutputs);  
    newDrone = new Drone(tempRigidBody);
    return newDrone;
  }
  
  void addSquadMate(Drone newDrone)        //adds new drones to the squad
  {
    if(members == null) {members = new Drone[1]; members[0] = newDrone;}
    else{members = (Drone[])append(members, newDrone);}
    members[members.length - 1].squad = this;
    int tempAdjust[] = calcFormationPosition();
    arrayCopy(tempAdjust, newDrone.coordinatesAdjusts);
  }
  
  //adds new drones to the squad, takes in adjusts as well which cancels out a call to the formation function, used to add special drones to the squad
  //note this extra member is not counted in members to avoid being caught up in rearrangement
  void addSquadMate(Drone newDrone, int[] adjusts)      
  {
    newDrone.squad = this;
    arrayCopy(adjusts, members[members.length - 1].coordinatesAdjusts);
  }
  
  Drone removeSquadMate()                                 //kicks the last joined member out of the squad and returns the drone object
  {
    Drone tempDrone = null;
    if(members == null || members.length == 0) {return tempDrone;}
    tempDrone = members[members.length - 1];
    members = (Drone[])shorten(members);
    reverseFormationFunction();                           //updates the status of the formation function and recalculates outershell shape
    tempDrone.squad = null;
    return tempDrone;  
  }
 
 int transferDrone(Squad squad2) //transfers a drone from squad1 to squad 2
 {
  if(members == null || members.length == 0) {return 0;}
  Drone tempDrone = removeSquadMate();
  squad2.addSquadMate(tempDrone);
  return 1;
 }
  
  void updateCoordinates()                    //easy way to call the update function for all the drones in a squad
  {
    squadLeader.updateCoordinates();
    if(members != null) {for(int i = 0; i < members.length; i++) {members[i].updateCoordinates();}}
  }
 
 //asigns the squad a new leader, and reforms the squad around the new leader
 void newLeader(Drone newLeader)
 {
  Drone oldLeader = squadLeader;                                                                         //saving him so we can add him back to the squad
  oldLeader.isLeader = false;
  newLeader.isLeader = true;
  newLeader.squad = this;
  this.squadLeader = newLeader;
  Drone []oldMembers = resetFormation();
  addSquadMate(oldLeader);
  
  boolean isInSquad = false;                                                                                      //if the new squadLeader was a member of the squad we will reform differently
  for(int i = 0; i < oldMembers.length; i++) {if(oldMembers[i] == squadLeader) {isInSquad = true;}}
  
  if(isInSquad) {for(int i = 0; i < oldMembers.length; i++) {if(oldMembers[i] == squadLeader) {continue;} addSquadMate(oldMembers[i]);}} //just keep everyone in nearly the same spot if the new leader was a member
  else {for(int i = oldMembers.length - 1; i > 0; i--) {if(oldMembers[i] == squadLeader) {continue;} addSquadMate(oldMembers[i]);}}  //peel off like an onion and switch spots outside to in and vice versa if the new leader was not a squad member
  
 }
  
 Drone[] resetFormation()    //quick way to reset the formation function whenever needed
 {
  shellNum = 0;
  maxDronesInShell = originalMax;
  numDronesInShell = 0;
  Drone[] oldMembers = new Drone[members.length];                                                        //remembering the old members so we can add them all back in new formation
  arrayCopy(members, oldMembers);
  members = null;
  return oldMembers;
 }
  
 void move(int coor[])                                                                         //resets the desired state of the leader so it will move there
 {
     for(int i = 0; i < NUM_AXIS; i++) {squadLeader.droneBody.desiredState[i] = coor[i];}
     squadLeader.droneBody.desiredStateReset = true;
 }
  
 void spaceOutDronesInOuterShell()                                                                              //spaces out everyone in the outer shell when not filled to maximize space efficiency
 {
    float mag = shellNum * shellRadius + shellRadius;
    int tempAdjusts[] = new int[NUM_AXIS];
    for(int j = 0; j < NUM_AXIS; j++) {tempAdjusts[j] = 0;}
    for(int i = members.length - 1; i > members.length - 1 - numDronesInShell; i--)
    {
      float angle = map(i - members.length + numDronesInShell, 0, numDronesInShell, 0, 2 * PI);
      tempAdjusts[0] = int(mag * cos(angle));
      tempAdjusts[1] = int(mag * sin(angle));
      arrayCopy(tempAdjusts, members[i].coordinatesAdjusts);
    }
 }
  
 void squadDebug(char[] startTitle)                                                          //just a testing function used to debug the formation function
 {
   println(startTitle);
   println("Shell Num: ", shellNum);
   println("Num In Shell: ", numDronesInShell);
   println("Shell Max: ", maxDronesInShell);
   println("\n\n");
 }
  
};