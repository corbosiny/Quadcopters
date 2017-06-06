int baseCoordinates[] = {250, 250};
int baseCoordinates2[] = {350, 350};

float forces[] = {0,0,0};
float constants[] = {5,5,.5};

TestDrone leadDrone = new TestDrone(new Agent(baseCoordinates, forces, constants, color(0,0,255)));
TestDrone leadDrone2 = new TestDrone(new Agent(baseCoordinates2, forces, constants, color(255,0,0)));

Drone[] squadDrones = new Drone[4];
Squad testSquad;
Squad testSquad2;

void setup()
{
  surface.setTitle("Swarm Simulation");
  //fullScreen();
  size(500, 500);
  surface.setResizable(true);
  //for(int i = 0; i < 4; i++) 
  //{
  //  int tempCoordinates[] = {50 * i, 50};
  //  int tempAdjust[] = {10+10 * i, 10+10 * i};
  //  squadDrones[i] = new Drone(new Agent(tempCoordinates, forces, constants), tempAdjust);
  //}

  testSquad = new Squad(leadDrone);
  //testSquad.createSquadMate();
  testSquad2 = new Squad(leadDrone2);
}

void draw()  //redraws the squads every turn
{
     background(0);
     //leadDrone.update();
     testSquad.update();
     //leadDrone2.update();
     testSquad2.update();
}

void mouseClicked() //just allows us to tell our squads where to move
{
 if(mouseButton == LEFT) {testSquad.move(mouseX, mouseY);} 
 if(mouseButton == RIGHT) {leadDrone2.move(mouseX, mouseY);}
} 




//key controls for the user to edit the world
void keyPressed()
{
 
  if(key == 'z') //transfers a drone from squad 1 to squad 2 if any
  {
     if(testSquad.members != null && testSquad.members.length != 0) //32 S
     {
       Drone tempDrone = testSquad.removeSquadMate();
       int tempAdjusts[] = testSquad2.formationFunction();
       tempDrone.adjusts = tempAdjusts;
       testSquad2.addSquadMate(tempDrone);
     }  
  }
  else if(key == 'x') //transfers a drone from squad 2 to squad 1 if any
  {
     if(testSquad2.members != null && testSquad2.members.length != 0)
     {
       Drone tempDrone = testSquad2.removeSquadMate();
       int tempAdjusts[] = testSquad.formationFunction();
       tempDrone.adjusts = tempAdjusts;
       testSquad.addSquadMate(tempDrone);
     } 
 }
 
 else if(key == 'c') //spawns a new drone in squad 1
 {
    testSquad.createSquadMate();
 }
 else if(key == 'v') //spawns a new drone in squad 2
 {
    testSquad2.createSquadMate();
 }
 
  
}