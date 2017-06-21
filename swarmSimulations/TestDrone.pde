class TestDrone extends Drone
{

    TestDrone(Agent droneBody) {super(droneBody);}
    boolean squadReset = false;    
    void randomMove() //randomly moves the drone leader around
    {
      
     droneBody.desiredState[0] += int(random(-5,5));
     droneBody.desiredState[1] += int(random(-5,5)); 
         
    }
    
    void move(int x, int y)  //resets the desired state so the PIDS will move the drone
    {
       droneBody.desiredState[0] = x;
       droneBody.desiredState[1] = y;
       droneBody.reset = true;
    }
    
    void update() {droneBody.applyForce();} //calculates PID forces and envionrment forces and determines what direction the drone will move
  
}