int messageCount = 0;

uint32_t timeCount = 0;
float p,r,y;

void sendDataPacket(char param, float value);

void setup() 
{
  pinMode(13, OUTPUT);
  Serial.begin(9600);
}

void loop() 
{
  if(Serial.available())
  {
      while(Serial.available())
      {
        char message = Serial.read();
        if(message == '1')
        {
          digitalWrite(13, HIGH);
        }
        else if(message == '0')
        {
          digitalWrite(13, LOW);
        }
      }
      
    Serial.print("Message Count: ");
    Serial.println(++messageCount);
  }

  sendDataPacket('p', p);
  delay(10);
  sendDataPacket('r', r);
  delay(10);
  sendDataPacket('y', y);
  delay(10);
  
  p += .0125;
  r += .0075;
  y += .0200;

  sendDataPacket('u', 3.0);
  delay(10);
  sendDataPacket('i', 4.0);
  delay(10);
  sendDataPacket('d', 5.0);
  delay(100);
}

void sendDataPacket(char param, float value)
{
  Serial.print(param);
  Serial.print(':');
  Serial.println(value);
}

