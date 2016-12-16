//**********************************************************************
//*                          FEROMONE IMU                               *
//*              CALCULATES CURRENT STATE OF THE DRONE                  *
//*         PITCH, ROLL, YAW(soon), TEMPERATURE, ALTITUDE(soon)         *
//*                  Sub-team members: Corey, Ali                       *
//*ASK COREY OR ALI ABOUT THE MATH AND/OR CODE IF YOU HAVE ANY QUESTIONS*
//***********************************************************************
//
//Gyro/Accel    -   MPU6050
//Barometer     -   TBD
//Magenetometer -   TBD

//MPU6050 PINOUT(I2C):
//  VCC  -  5V
//  GND  -  GND
//  SDA  -  A4
//  SCL  -  A5

//Barameter PINOUT:
//  -

//Magenomter PINOUT:
//  -

#include <Wire.h>
#include <BMP085.h>

#include "I2Cdev.h"
#include "HMC5883L.h"

int gyroX, gyroY, gyroZ;                                                 //holds the rate of change the quadcopter's angles detected by the MPU6050
long int accX, accY, accZ, accMag;                                       //hold the acceleration values of their respective axsis and the magnitude
float anglePitch, angleRoll;                                             //first the gyro calculated angles, then the IMU pitch and roll values
float angleRollAcc, anglePitchAcc;                                       //calculated rotaion angles based off accelerometer
long gyroXoffset, gyroYoffet, gyroZoffset;                               //used for calibration of the initial gyro reading offsets
boolean setGyroAngles = 0;                                               //used to trigger initial angle readings, only is used on startup
int temperature;                                                         //self explanatory bruh

int MPUaddr = 0x68;                                                      //MPU address on the i2c bus, DO NOT CHANGE I WILL FIND YOU 
float accPitchOffset = 0, accRollOffset = 0;                             //offsets for the accelerometer values, laying it flat is the best way to calculate these
float gyroContribution = .9996;                                          //used for the complimentary filter, how much weight the gyro's measurments are given vs the accelerometer

BMP085 bmp;                                                              //The barometer object
int pressure;                                                            //holds the air pressure readings
long int altitude;                                                       //holds the current altitude
long int seaLevelPress;                                                  //holds the local air pressure at sea level(used for altitude), must be entered

HMC5883L magmtr;                                                         //The magentometer object
int16_t mx, my, mz;                                                      //holds magentic field values on each axis
float heading;                                                           //will store your angle in relation to the north pole, 0 = North

void setup() 
{

  Wire.begin();  
  bmp.begin();                                                            //starts up the barometer
  mag.initialize();                                                       //starts up the magentometer
  
  
  
  Serial.begin(9600);                                                                                       //Use serial only for debugging
  Serial.println("Testing device connections..."); 
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
  
  pinMode(13, OUTPUT);                                                   //if this pin is in use take out this line and the digital writes to it below
  
  setupMPU6050Registers();                                               //Setup the registers of the MPU-6050, look at datasheet for more info

  digitalWrite(13, HIGH);                                                
  
  for (int i; i < 2000 ; i++)                                            //here we will calculate the offsets for our gyro as they can drift
  {                                           
      
      read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU6050
      gyroXoffset += gyroX;                                              //keep running total of 2000 readings
      gyroYoffset += gyroY;                                              
      gyroZoffset += gyroZ;                                              
      delay(3);                                                          
  
  }
  
  Serial.println("Done calibrating");
  gyroXoffset /= 2000;                                                  //Divide the gyro variables by 2000 to get the avarage offset
  gyroYoffset /= 2000;                                                  
  gyroZoffset /= 2000;                                                  

  digitalWrite(13, LOW);                                               
                                                                         
}

void loop()
{

    readMPU6050data();                                                                      //loads in all the raw readings from the MPU6050

    gyroX -= gyroXoffset;                                                                    //Subtracting the calibration offset from the raw gyro values
    gyroY -= gyroYoffset;                                                
    gyroZ -= gyroZoffset;                                                

    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)
    anglePitch += gyroX * 0.0000611;                                                        //Calculate the traveled pitch angle and add this to the angle_pitch variable
    angleRoll += gyroY * 0.0000611;                                                         //Calculate the traveled roll angle and add this to the angle_roll variable

    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) to convert to radians
    anglePitch += angleRoll * sin(gyroZ * 0.000001066);                                     //If the IMU is rotating, we must transfer the pitch and the roll between eachother in a sinosoidal wave form
    angleRoll -= anglePitch * sin(gyroZ * 0.000001066);                                     //this isn't the easiest concept to explain in comments, feel free to ask Corey

    //Accelerometer angle calculations
    accMag = sqrt(sq(accX)+sq(accY)+sq(accZ));                                              //Calculate the total accelerometer vector magnitude

    //57.296 = 1 / (3.142 / 180) to convert to radians
    anglePitchAcc = asin((float)accY/accMag)* 57.296;                                       //Calculate the pitch angle
    angleRollAcc = asin((float)accX/accMag)* -57.296;                                       //Calculate the roll angle

    //set unit flat to determine these values
    //helps to determine initial angles better
    anglePitchAcc -= accPitchOffset;                                                        //Accelerometer calibration value for pitch
    angleRollAcc -=  accRollOffset;                                               


    //below we use acomplimentary filter
    //basically a weighted average of gyro and accelerometer readings
    //result is essentially a bandpass filter that disregards random small spikes in accel readings
    //and corrects long term drift of the gyro, sort of like an integral proportional controller but averaged

    if(setGyroAngles)                                                                             //If the IMU is already started
    {                                                 
        anglePitch = anglePitch * gyroContribution + anglePitchAcc * (1 - gyroContribution);     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
        angleRoll = angleRoll * gyroContribution + angleRollAcc * (1 - gyroContribution);        
    }
    else
    {                                                                                         //Only occurs first start
        anglePitch = anglePitchAcc;                                                           //Set the initial angles to the accelerometer angles 
        angleRoll = angleRollAcc;                                        
        setGyroAngles = true;                                                                 //prevents any more triggering
    }

    temperature = temperature / 340.00 + 36.53;                                               //from data sheet, converts temp reading to celcius
    
    Serial.print("Pitch: ");                                                                  //For Debugging Purposes
    Serial.println(angle_pitch);
    Serial.print("Roll: ");
    Serial.println(angle_roll);
    Serial.print("Temperature: ")
    Serial.println(temperature);
    Serial.print("Air Pressure: ")
    Serial.println(pressure);
    Serial.print("Altitude: ")
    Serial.println(altitude);
    Serial.print("Heading(0 = North): ");
    Serial.println(heading);
    Serial.println("\n\n");
  
}

void readBMPdata()                                                                               //Reads in and fills all the variables that hold BMP data
{
  
  pressure = bmp.readPressure()
  altitude = bmp.readAltitude(seaLevelPress);                                                     //uses local sea level air pressure to calculatre altitude
  
}

void readHMCdata()                                                                                //Acts like a compass and calcualtes the drones bearing due north
{

   mag.getHeading(&mx, &my, &mz);                                                                  //gets the magnetic field strengths on each asix
   float heading = atan2(my, mx);                                                                  //looks at x and y to determine the overall direction to the north pole
   if(heading < 0) {heading += 2 * M_PI;}                                                          //Turns a negative angle positive
   heading *= (180 / M_PI);                                                                        //Turns the angle from radians to degrees
  
}

//--------------------------------------------------------------------------------------------------------
//-!!!!!!!!!!!!DONT EVER TOUCH THE CODE BELOW THIS EVER, I SWEAR THERE'LL BE HELL TO PAY!!!!!!!!!!!!!!!!!-
//-------------                                    From, Corey                           -----------------
//--------------------------------------------------------------------------------------------------------
void readMPU6050data()                                                 //loads in all the readings from the corresponding registers in the MPU
{                                                                      //all this comes from the data sheet 
                                                                       //ali, if you see this comment send me a text saying hi
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  accX = Wire.read()<<8|Wire.read();                                   //Add the low and high byte to the acc_x variable
  accY = Wire.read()<<8|Wire.read();                                   //Add the low and high byte to the acc_y variable
  accZ = Wire.read()<<8|Wire.read();                                   //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyroX = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the gyro_x variable
  gyroY = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the gyro_y variable
  gyroZ = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the gyro_z variable

}

void setupMPU6050Registers()
{
  
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission

}
