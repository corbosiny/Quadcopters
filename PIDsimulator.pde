int baseCoordinates[] = {50, 250};
int baseCoordinates2[] = {100, 250};
int baseCoordinates3[] = {150, 250};
int baseCoordinates4[] = {200, 250};
float constants[] = {0,0,0};
float constants2[] = {2, 0, 0};
float constants3[] = {2, 2, 0};
float constants4[] = {2, 2, .2};

Agent agents1 = new Agent(baseCoordinates,-80, constants);
Agent agents2 = new Agent(baseCoordinates2, -80, constants2);
Agent agents3 = new Agent(baseCoordinates3, -80, constants3);
Agent agents4 = new Agent(baseCoordinates4,  -80, constants4);
Agent agents[] = {agents1, agents2, agents3, agents4};

void setup()
{
  size(500, 500);
}

void draw()
{
  
  background(0);
  for(int i = 0; i < agents.length; i++) {agents[i].applyForce();}

}