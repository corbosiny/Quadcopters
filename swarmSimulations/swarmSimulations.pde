int baseCoordinates[] = {250, 250};
int baseCoordinates2[] = {350, 350};

float forces[] = {0,0,0};
float forces2[] = {0,0,0};
float constants[] = {5,5,.3};
int maxOutputs[] = {150, 250, 100};

int maxDistance = 25;
int minDistance = 10;

int NUM_AXIS = 2;

Agent agents[] = new Agent[0];

TestDrone leadDrone = new TestDrone(new Agent(baseCoordinates, forces, constants, maxOutputs, color(0,0,255)));
TestDrone leadDrone2 = new TestDrone(new Agent(baseCoordinates2, forces2, constants, maxOutputs, color(255,0,0)));

Squad testSquad;
Squad testSquad2;

Squad squads[] = new Squad[0];

void setup()
{
  surface.setTitle("Swarm Simulation");
  //fullScreen();
  size(750, 400);
  surface.setResizable(true);

  testSquad = new Squad(leadDrone, 10, 30);
  testSquad2 = new Squad(leadDrone2, 8, 20);
}

//redraws the squads every turn
void draw()
{
     background(0);
     for(int i = 0; i < squads.length; i++)
     {
        squads[i].update();
        if(squads[i].members != null)
        {
            for(int j = 0; j < squads[i].members.length; j++)
            {
              squads[i].members[j].droneBody.forces = generateForces(20);
            }
        } 
     }
     
}


//just allows us to tell our squads where to move
void mouseClicked()
{
 if(mouseButton == LEFT) {testSquad.move(mouseX, mouseY);} 
 if(mouseButton == RIGHT) {leadDrone2.move(mouseX, mouseY);}
} 



//key controls for the user to edit the world
void keyPressed()
{
  if(key == 'z')                                                 //transfers a drone from squad 1 to squad 2 if any
  {testSquad.transferDrone(testSquad2);}
  else if(key == 'x')                                            //transfers a drone from squad 2 to squad 1 if any
  {testSquad2.transferDrone(testSquad);}
 
 else if(key == 'c')                                             //spawns a new drone in squad 1
 {testSquad.addSquadMate(testSquad.createSquadMate());}
 else if(key == 'v')                                             //spawns a new drone in squad 2
 {testSquad2.addSquadMate(testSquad2.createSquadMate());}

 else if(key == 'w') 
 {
   if(testSquad.members != null) {for(int i = 0; i < testSquad.members.length; i++) {testSquad.members[i].droneBody.forces[1] -= 10;}}
   testSquad.squadLeader.droneBody.forces[1] -= 10;
 }
 else if(key == 's') 
 {
   if(testSquad.members != null) {for(int i = 0; i < testSquad.members.length; i++) {testSquad.members[i].droneBody.forces[1] += 10;}}
   testSquad.squadLeader.droneBody.forces[1] += 10;
 }
 else if(key == 'a') 
 {
   if(testSquad.members != null) {for(int i = 0; i < testSquad.members.length; i++) {testSquad.members[i].droneBody.forces[0] -= 10;}}
   testSquad.squadLeader.droneBody.forces[0] -= 10;
 }
 else if(key == 'd') 
 {
   if(testSquad.members != null) {for(int i = 0; i < testSquad.members.length; i++) {testSquad.members[i].droneBody.forces[0] += 10;}}
   testSquad.squadLeader.droneBody.forces[0] += 10;
 }
 

}

float []generateForces(int max)
{
 
 float []forces = new float[NUM_AXIS];
 for(int i = 0; i < NUM_AXIS; i++)
 {
   forces[i] = int(random(-max, max));
 }
 return forces;
  
}

float []generateForces(int max[])
{
  float []forces = new float[NUM_AXIS];
  for(int i = 0; i < NUM_AXIS; i++)
  {
   forces[i] = int(random(-max[i], max[i])); 
  }
  return forces;  
}

float []generateTrigForces(int mag)
{
  float []forces = new float[NUM_AXIS];
  for(int i = 0; i < NUM_AXIS; i++)
  {
    forces[i] = int(mag * cos(millis()));
  }
  return forces;
}

int []generateTrigForces(int mag[])
{
 int []forces = new int[NUM_AXIS]; 
 for(int i = 0; i < NUM_AXIS; i++)
 {
   forces[i] = int(mag[i] * cos(millis())); 
 }
 return forces;
}