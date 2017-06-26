#include "IMU.h"

IMU::IMU(long int seaLPress, float gyroCon = .9996, float accPO = 0, float accRO = 0, int reRate = 250)
{

  Wire.begin();                                         //intializing our librarites and modules
  bmp.initialize();                                     //starts up the barometer
  bmp.setControl(BMP085_MODE_PRESSURE_1);                                               
  mag.initialize();

  seaLevelPress = seaLPress;                            //Local air pressure at sea pressure, must be looked up
  gyroContribution = gyroCon;                           //how much the gyro angle contributes in the complimentary filter
  accPitchOffset = accPO;                                //accelerometer pitch offset
  accRollOffset = accRO;                                 //accelerometer roll offset
  refreshRate = reRate;                                 //the refresh rate is defaulted to 250 Hz but should be measured for more accuracy
  
  setupMPU6050Registers();  
  calibrateGyro();    
  
}

void IMU::setupMPU6050Registers()
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

void IMU::calibrateGyro()                                              //Calibrates Gyro Offsets
{

  Wire.begin();  
  bmp.initialize();                                                      //starts up the barometer
  bmp.setControl(BMP085_MODE_PRESSURE_1);                                //sets the barometer to standard quality measurment mode(4 modes) ranging from low power to high quality
  mag.initialize();                                                      //starts up the magentometer
  
  pinMode(13, OUTPUT);                                                   //if this pin is in use take out this line and the digital writes to it below
  digitalWrite(13, HIGH);                                                
  
  for (int i; i < 2000 ; i++)                                            //here we will calculate the offsets for our gyro as they can drift
  {                                           
      
      readMPUData();                                              //Read the raw acc and gyro data from the MPU6050
      gyroXoffset += gyroX;                                              //keep running total of 2000 readings
      gyroYoffset += gyroY;                                              
      gyroZoffset += gyroZ;                                              
      delay(3);                                                          
  
  }
  
  gyroXoffset /= 2000;                                                  //Divide the gyro variables by 2000 to get the avarage offset
  gyroYoffset /= 2000;                                                  
  gyroZoffset /= 2000;                                                  

  digitalWrite(13, LOW);  
  
}

float *IMU::readIMUData()
{

  readMPUData();                                                                      //functions to read all the data we need from the IMU
  readBMPData();                                                                          //each function deals with a respective module of the IMU
  readMCLData();  
  float values[] = {anglePitch, angleRoll, heading, alt};
  return values;
    
}

void IMU::readMPUDataRaw()
{

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

void IMU::readMPUData()
{

    readMPUDataRaw();
    gyroX -= gyroXoffset;                                                                    //Subtracting the calibration offset from the raw gyro values
    gyroY -= gyroYoffset;                                                
    gyroZ -= gyroZoffset;                                                

    //Gyro angle calculations
    //0.0000611 = 1 / (default refreshRate / 65.5)
    int conversion = 1.0 / refreshRate / 65.5;
    anglePitch += gyroX * conversion;                                                        //Calculate the traveled pitch angle and add this to the angle_pitch variable
    angleRoll += gyroY * conversion;                                                         //Calculate the traveled roll angle and add this to the angle_roll variable

    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) to convert to radians
    anglePitch += angleRoll * sin(gyroZ * conversion * (3.142 / 180.0));                       //If the IMU is rotating, we must transfer the pitch and the roll between eachother in a sinosoidal wave form
    angleRoll -= anglePitch * sin(gyroZ * conversion * (3.142 / 180.0));                       //this isn't the easiest concept to explain in comments, feel free to ask Corey

    //Accelerometer angle calculations
    accMag = sqrt((accX * accX) + (accY * accY) + (accZ * accZ));                                              //Calculate the total accelerometer vector magnitude

    //57.296 = 1 / (3.142 / 180) to convert to radians
    anglePitchAcc = asin((float)accY/accMag)* 57.296;                                       //Calculate the pitch angle
    angleRollAcc = asin((float)accX/accMag)* -57.296;                                       //Calculate the roll angle

    //set unit flat to determine these values
    //helps to determine initial angles better
    anglePitchAcc -= accPitchOffset;                                                        //Accelerometer calibration value for pitch
    angleRollAcc -=  accRollOffset;                                               


    //below we use a complimentary filter
    //basically a weighted average of gyro and accelerometer readings
    //result is essentially a bandpass filter that disregards random small spikes in accel readings
    //and corrects long term drift of the gyro, sort of like an integral proportional controller but averaged

    if(setGyroAngles)                                                                             //If the IMU is already started
    {                                                 
        anglePitch = anglePitch * gyroContribution + anglePitchAcc * (1.0 - gyroContribution);     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
        angleRoll = angleRoll * gyroContribution + angleRollAcc * (1.0 - gyroContribution);        
    }
    else
    {                                                                                         //Only occurs first start
        anglePitch = anglePitchAcc;                                                           //Set the initial angles to the accelerometer angles 
        angleRoll = angleRollAcc;                                        
        setGyroAngles = true;                                                                 //prevents any more triggering
    }

    temperature = temperature / 340.00 + 36.53;                                               //from data sheet, converts temp reading to celcius    
    
}

void IMU::readBMPData()
{
  pressure = bmp.getPressure();                                        
  alt = bmp.getAltitude(seaLevelPress);                           //uses local sea level air pressure to calculatre altitude
}

void IMU::readMCLData()
{
   mag.getHeading(&mx, &my, &mz);                                         //gets the magnetic field strengths on each asix
   heading = atan2(my, mx);                                               //looks at x and y to determine the overall direction to the north pole
   if(heading < 0) {heading += 2.0 * M_PI;}                               //Turns a negative angle positive
   heading *= (180.0 / M_PI);                                             //Turns the angle from radians to degrees
}
