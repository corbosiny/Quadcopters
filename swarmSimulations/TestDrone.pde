class TestDrone extends Drone
{

    TestDrone(Agent droneBody) {super(droneBody);}
    
    void randomMove()
    {
      
     droneBody.desiredState[0] += int(random(-5,5));
     droneBody.desiredState[1] += int(random(-5,5)); 
         
    }
    
    void move(int x, int y)
    {
       droneBody.desiredState[0] = x;
       droneBody.desiredState[1] = y;
       droneBody.reset = true;
    }
    
    void update() {droneBody.applyForce();}
  
}