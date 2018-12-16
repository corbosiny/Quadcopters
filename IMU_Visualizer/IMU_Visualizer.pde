import processing.serial.*;

Serial myPort;  // Create object from Serial class
String val;     // Data received from the serial port

int h = 1000;
int w = 520; 

void setup()
{
  String portName = Serial.list()[0]; 
  println(portName);
  myPort = new Serial(this, portName, 9600);

  size(1000, 520, P3D);
  noStroke();
  fill(204);
}

float fov = PI/3.0;
float cameraZ = (height / 2.0) / tan(fov / 2.0);
float horizontalRotation = 0;
float verticalRotation = 0;
float zoom = (height / 2) / tan(PI / 6.0) + 200;

float p, pIncrem;
float r, rIncrem;
float y, yIncrem;
void draw()
{
  /*background(0);
  camera(mouseX + horizontalRotation, mouseY + verticalRotation, zoom, width/2, height/2, 0, 0, 1, 0);
  translate(width/2, height/2, -100);
  stroke(255);
  noFill();
  box(200);*/
  background(0);
  lights();
  smooth();
  camera(mouseX, mouseY, zoom, width/2, height/2, 0, 0, 1, 0);
  pushMatrix();
  noStroke();
  translate(width/2, height/2, 0);
  rotateY(p);
  rotateX(r);
  rotateZ(y);
  box(100);
  popMatrix();
  

}

String message = "";
void serialEvent(Serial port)
{
   if(port.available() > 0)
   {
     char lastChar = (char)port.read();
     message += lastChar;
     if(lastChar != '\n') {return;}
     
     if(message.length() <= 3) {return;} //<>//
     if(message.charAt(0) == 'p')
     {
       float value = float(message.substring(2));
       p += value;
       //print("value of p: ");
       //print(p);
     }
     else if(message.charAt(0) == 'r')
     {
       float value = float(message.substring(2));
       r += value;
     }
     else if(message.charAt(0) == 'y')
     {
       float value = float(message.substring(2));
       y += value;
     }
     message = "";
   } 
}

boolean front = true;
void keyPressed()
{
  
  if(key == CODED)
  {
    if(keyCode == UP)
    {
      if(front == true) {zoom -= 30;}
      else{zoom += 30;}
    }
    else if(keyCode == DOWN)
    {
      if(front == true) {zoom += 30;}
      else{zoom -= 30;}
    }

  }
  else
  {
    if(key == '1') {myPort.write('1');}
    else if(key == '0') {myPort.write('0');}
    else if(key == 'r') {reverseView();}    
  }
}

void reverseView()
{
      if(front)
      {
        front = false;
        zoom *= -1;
      }
      else
      {
        front = true;
        zoom *= -1;
      }
}