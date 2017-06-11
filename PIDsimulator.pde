//sets up coordinates, PID constants, and active forces for our agents
int baseCoordinates[] = {50, 250};
int baseCoordinates2[] = {100, 250};
int baseCoordinates3[] = {150, 250};
int baseCoordinates4[] = {200, 250};

float constants[] = {0,0,0};
float constants2[] = {4, 0, 0};
float constants3[] = {4, 10, 0};
float constants4[] = {4, 10, .3};
float forces[] = {-50, -50};

int maxOutputs[] = {400, 700, 200}; 
int maxDistance = 30;
int minDistance = 15;

//setting up our agents
Agent agents[] = new Agent[0];
Agent agents1 = new Agent(baseCoordinates, forces, constants, null, color(255, 255, 255));
Agent agents2 = new Agent(baseCoordinates2, forces, constants2, maxOutputs, color(255,0,0));
Agent agents3 = new Agent(baseCoordinates3, forces, constants3, maxOutputs, color(0,255,0));
Agent agents4 = new Agent(baseCoordinates4,  forces, constants4, maxOutputs, color(0,255,255));

void setup()
{
  size(500, 500); //just making our canvas
  surface.setResizable(true);
  frame.setTitle("PID Simulation");
}

void draw()
{
  
  background(0); //reset the background
  for(int i = 0; i < agents.length - 1; i ++) 
  {
    for(int j = i + 1; j < agents.length; j++)
    {
      for(int k = 0; k < 2; k++)
      {
        agents[i].forces[k] += agents[i].obstacleAvoidance(k, agents[i], agents[j]); 
        agents[j].forces[k] += agents[j].obstacleAvoidance(k, agents[i], agents[j]);
      }
    }
  
  }
  for(int i = 0; i < agents.length; i++) {agents[i].applyForce();} //calculate new states of the drones based off PID calcs and forces
  
}

void keyPressed()
{
 
    if(key == 'w') //increase force in up y direction, negative force is up
    {for(int i = 0; i < agents.length; i++) {agents[i].forces[1] -= 10;}}
    
    else if(key == 's') //increase force in down y direction, negative force is up
    {for(int i = 0; i < agents.length; i++) {agents[i].forces[1] += 10;}}


    if(key == 'a')  //increase force in left x direction, negative force is left
    {
       for(int i = 0; i < agents.length; i++) {agents[i].forces[0] -= 10;}
    }
    else if(key == 'd') //increase force in right x direction, negative force is left
    {
       for(int i = 0; i < agents.length; i++) {agents[i].forces[0] += 10;} 
    }
    
}

void mouseClicked() //sets new desired state for agents
{
  
  for(int i = 0; i < agents.length; i++)
  {
    agents[i].desiredState[0] = mouseX; //set the new desired state for each agent to the mouse coordinates
    agents[i].desiredState[1] = mouseY;
    agents[i].reset = true;  //sets reset so we can reset our derivative and integral term to true so that we don't have a spike in the derivative term
  } 

}