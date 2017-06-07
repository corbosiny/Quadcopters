int baseCoordinates[] = {250, 250};
int baseCoordinates2[] = {350, 350};

float forces[] = {0,0,0};
float constants[] = {5,5,.5};

TestDrone leadDrone = new TestDrone(new Agent(baseCoordinates, forces, constants, color(0,0,255)));
TestDrone leadDrone2 = new TestDrone(new Agent(baseCoordinates2, forces, constants, color(255,0,0)));

Squad testSquad;
Squad testSquad2;

void setup()
{
  surface.setTitle("Swarm Simulation");
  fullScreen();
  surface.setResizable(true);

  testSquad = new Squad(leadDrone);
  testSquad2 = new Squad(leadDrone2);
  
}

//redraws the squads every turn
void draw()
{
     background(0);
     testSquad.update();
     testSquad2.update();
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
}