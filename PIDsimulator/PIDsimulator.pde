//sets up coordinates, PID PIDconstants, and active outsideForces for our rigidBodies
int baseCoordinates[] = {50, 250};
int baseCoordinates2[] = {100, 250};
int baseCoordinates3[] = {150, 250};
int baseCoordinates4[] = {200, 250};

float PIDconstants[] = {0, 0, 0};
float PIDconstants2[] = {4, 0, 0};
float PIDconstants3[] = {4, 10, 0};
float PIDconstants4[] = {4, 10, .8};
float outsideForces[] = {-50, -50};

int maxPIDoutputs[] = {300, 400, 200}; 
int maxDistance = 50;
int minDistance = 10;

//setting up our rigidBodies
RigidBody rigidBodies[] = new RigidBody[0];
RigidBody rigidBodies1 = new RigidBody(baseCoordinates, outsideForces, PIDconstants, maxPIDoutputs, color(255, 255, 255));
RigidBody rigidBodies2 = new RigidBody(baseCoordinates2, outsideForces, PIDconstants2, maxPIDoutputs, color(255, 0, 0));
RigidBody rigidBodies3 = new RigidBody(baseCoordinates3, outsideForces, PIDconstants3, maxPIDoutputs, color(0, 255, 0));
RigidBody rigidBodies4 = new RigidBody(baseCoordinates4, outsideForces, PIDconstants4, maxPIDoutputs, color(0, 0, 255));

void setup()
{
  size(500, 500); //just making our canvas
  surface.setResizable(true);
  surface.setTitle("PID Simulation");  
}

void draw()
{

  
  background(0); 
  for (int i = 0; i < rigidBodies.length; i++) {
    rigidBodies[i].applyForces();
  } 
}

void keyPressed()
{

  if (key == 'w') //increase force in up y direction, negative force is up
  {
    for (int i = 0; i < rigidBodies.length; i++) {
      rigidBodies[i].outsideForces[1] -= 10;
    }
  } else if (key == 's') //increase force in down y direction, negative force is up
  {
    for (int i = 0; i < rigidBodies.length; i++) {
      rigidBodies[i].outsideForces[1] += 10;
    }
  }


  if (key == 'a')  //increase force in left x direction, negative force is left
  {
    for (int i = 0; i < rigidBodies.length; i++) {
      rigidBodies[i].outsideForces[0] -= 10;
    }
  } else if (key == 'd') //increase force in right x direction, negative force is left
  {
    for (int i = 0; i < rigidBodies.length; i++) {
      rigidBodies[i].outsideForces[0] += 10;
    }
  }
}

void mouseClicked() //sets new desired state for rigidBodies
{

  if(mouseButton == LEFT)
  {
    for (int i = 0; i < rigidBodies.length - 1; i++)
    {
      rigidBodies[i].desiredState[0] = mouseX; //set the new desired state for each agent to the mouse coordinates
      rigidBodies[i].desiredState[1] = mouseY;
      rigidBodies[i].desiredStateReset = true;  //sets reset so we can reset our derivative and integral term to true so that we don't have a spike in the derivative term
    }
  }
  else if(mouseButton == RIGHT)
  {
      rigidBodies[rigidBodies.length - 1].desiredState[0] = mouseX; //set the new desired state for each agent to the mouse coordinates
      rigidBodies[rigidBodies.length - 1].desiredState[1] = mouseY;
      rigidBodies[rigidBodies.length - 1].desiredStateReset = true;  //sets reset so we can reset our derivative and integral term to true so that we don't have a spike in the derivative term  
  }

}