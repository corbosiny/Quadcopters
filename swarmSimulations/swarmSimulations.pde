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
  size(500, 500);
  surface.setTitle("Swarm Simulation");
  for(int i = 0; i < 4; i++) 
  {
    int tempCoordinates[] = {50 * i, 50};
    int tempAdjust[] = {10+10 * i, 10+10 * i};
    squadDrones[i] = new Drone(new Agent(tempCoordinates, forces, constants), tempAdjust);
  }

  testSquad = new Squad(leadDrone, squadDrones);
  //testSquad.createSquadMate();
  testSquad2 = new Squad(leadDrone2);
}

void draw()
{
     background(0);
     //leadDrone.update();
     testSquad.update();
     //leadDrone2.update();
     testSquad2.update();
}

void mouseClicked()
{
 if(mouseButton == LEFT) {testSquad.move(mouseX, mouseY);} 
 if(mouseButton == RIGHT) {leadDrone2.move(mouseX, mouseY);}
} 

void keyPressed()
{
 
  if(key == 'z') 
  {
     if(testSquad.members != null && testSquad.members.length != 0)
     {
       Drone tempDrone = testSquad.members[testSquad.members.length - 1];
       testSquad.members = (Drone[])shorten(testSquad.members);
       testSquad2.addSquadMate(tempDrone);
     }  
  }
  else if(key == 'x')
  {
     if(testSquad2.members != null && testSquad2.members.length != 0)
     {
       Drone tempDrone = testSquad2.members[testSquad2.members.length - 1];
       testSquad2.members = (Drone[])shorten(testSquad2.members);
       testSquad.addSquadMate(tempDrone);
     } 
 }
 
 else if(key == 'c')
 {
    testSquad.createSquadMate();
 }
 else if(key == 'v')
 {
    testSquad2.createSquadMate();
 }
 
  
}