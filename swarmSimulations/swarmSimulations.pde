int baseCoordinates[] = {250, 250};                                                                                          //starting coordinates for the first squad leader
int baseCoordinates2[] = {350, 350};                                                                                         //starting coordinates for the second squad leader

float outsideForces[] = {0,0};                                                                                                      //starting forces on the first squad leader
float outsideForces2[] = {0,0};                                                                                                     //starting forces on the second squad leader
float PIDconstants[] = {15,10,.5};                                                                                                //PID constants 1 - proportional, 2 - integral, 3 - derivative
int maxPIDoutputs[] = {150, 250, 100};                                                                                          //Speed limits for the PID controller outputs

int maxDistance = 30;                                                                                                        //distance the obstalce avoidance starts at
int minDistance = 10;                                                                                                        //minimum distance a drone will get to an object

int NUM_AXIS = 2;                                                                                                            //number of dimensions, program is scalable to multiple dimensions

RigidBody rigidBodies[] = new RigidBody[0];                                                                                               //holds a list of all the rigid bodies in the field, used for obstacle avoidance

Squad testSquad;                                                                                                             
Squad testSquad2;

int adjusts[] = new int[NUM_AXIS];

Drone leadDrone = new Drone(new RigidBody(baseCoordinates, outsideForces, PIDconstants, maxPIDoutputs, color(0,0,255)), testSquad);              //creating our two initial squad leaders
Drone leadDrone2 = new Drone(new RigidBody(baseCoordinates2, outsideForces2, PIDconstants, maxPIDoutputs, color(255,0,0)), testSquad2);

Squad squads[] = new Squad[0];                                                                                               //keeps track of all the squads in the simulation

void setup()
{
  surface.setTitle("Swarm Simulation");                           
  fullScreen();
  //size(750, 400);
  surface.setResizable(true);                                                                                                

  testSquad = new Squad(leadDrone);                                                                     //initializing our squads, takes a leader, number of drones that fit in the first shell of the formation, and the radius of the first shell; here I choose a raidus just outside the distance the drones start avoidong obstacles 
  testSquad2 = new Squad(leadDrone2);
}

//redraws the squads every turn
void draw()
{
     background(0);
     for(int i = 0; i < squads.length; i++)
     {
        squads[i].updateCoordinates();                                                                                     //applies forces and calculates new coordiantes for the squad leaders
        if(squads[i].members != null)
        {
            for(int j = 0; j < squads[i].members.length; j++)
            {
              squads[i].members[j].droneBody.outsideForces = generateRandomOutsideForces(20);                              //applies random forces to each member in the squad if a list of members exist
            }
        } 
     }
     
}


//just allows us to tell our squads where to move, left click sets a new point for squad 1, right click sets a new point for squad 2
void mouseClicked()
{
 int coor[] = {mouseX, mouseY};
 if(mouseButton == LEFT) {testSquad.move(coor);}           
 if(mouseButton == RIGHT) {testSquad2.move(coor);}
} 



//key controls for the user to edit the world
void keyPressed()
{
  if(key == 'z')                                                 //transfers a drone from squad 1 to squad 2 if any
  {testSquad.transferDrone(testSquad2);}
  else if(key == 'x')                                            //transfers a drone from squad 2 to squad 1 if any
  {testSquad2.transferDrone(testSquad);}
 
 else if(key == 'c')                                             
 {testSquad.addSquadMate(testSquad.createSquadMate());}
 else if(key == 'v')                                             
 {testSquad2.addSquadMate(testSquad2.createSquadMate());}

  else if(key == '0')       //used for testing leader switching, sets the new leader to the first member in the squad
  {
    if(testSquad.members != null && testSquad.members.length > 0) {testSquad.newLeader(testSquad.members[0]);}
  }


 //all keys below increase forces on squad 1, so that you can see the effect of forces on a squad vs no forces on another squad(squad 2 in this case)
 else if(key == 'w')  //increases forces north
 {
   if(testSquad.members != null) {for(int i = 0; i < testSquad.members.length; i++) {testSquad.members[i].droneBody.outsideForces[1] -= 10;}}
   testSquad.squadLeader.droneBody.outsideForces[1] -= 10;
 }
 else if(key == 's')  //increases forces south
 {
   if(testSquad.members != null) {for(int i = 0; i < testSquad.members.length; i++) {testSquad.members[i].droneBody.outsideForces[1] += 10;}}
   testSquad.squadLeader.droneBody.outsideForces[1] += 10;
 }
 else if(key == 'a') //increases forces west
 {
   if(testSquad.members != null) {for(int i = 0; i < testSquad.members.length; i++) {testSquad.members[i].droneBody.outsideForces[0] -= 10;}}
   testSquad.squadLeader.droneBody.outsideForces[0] -= 10;
 }
 else if(key == 'd') //increase forces east
 {
   if(testSquad.members != null) {for(int i = 0; i < testSquad.members.length; i++) {testSquad.members[i].droneBody.outsideForces[0] += 10;}}
   testSquad.squadLeader.droneBody.outsideForces[0] += 10;
 }
 

}

//all functions below here just generate random forces, can be substituted into the draw loop for the generate forces function to see the effect of different types of forces
float []generateRandomOutsideForces(int max) //generates a list of forces on each axis up to the max (can be positive or negative)
{
 
 float []forces = new float[NUM_AXIS];
 for(int i = 0; i < NUM_AXIS; i++)
 {
   forces[i] = int(random(-max, max));
 }
 return forces;
  
}

float []generateRandomOutsideForces(int max[]) //generates a list of the forces on each axis, this one can take a max for each axis (can be positive or negative)
{
  float []forces = new float[NUM_AXIS];
  for(int i = 0; i < NUM_AXIS; i++)
  {
   forces[i] = int(random(-max[i], max[i])); 
  }
  return forces;  
}

float []generateRandomTrigForces(int mag) //generates sinosoidal forces up to the given magnitude for each axis
{
  float []forces = new float[NUM_AXIS];
  for(int i = 0; i < NUM_AXIS; i++)
  {
    forces[i] = int(mag * cos(millis()));
  }
  return forces;
}

int []generateRandomTrigForces(int mag[]) //generates sinosoidal forces up to the given magnitude, can give a mag for each axis in this function
{
 int []forces = new int[NUM_AXIS]; 
 for(int i = 0; i < NUM_AXIS; i++)
 {
   forces[i] = int(mag[i] * cos(millis())); 
 }
 return forces;
}