#include "IMU.h"

IMU::IMU(float accelerometerPitchOffset = 0, float accelerometerRollOffset = 0, float gyroFilterWeight = .9996)
{
  this->gyroFilterWeight = gyroFilterWeight;            //how much the gyro angle contributes in the complimentary filter
  this->accelerometerPitchOffset = accelerometerPitchOffset;                               
  this->accelerometerRollOffset = accelerometerRollOffset;                                

  initializeIMUsensors();
  setupMPU6050Registers();  
  calibrateAndSetGyroSensorOffsets();   
}

void IMU::initializeIMUsensors()
{
  Wire.begin();  
  barometer.initialize();                                                      
  barometer.setControl(BMP085_MODE_PRESSURE_3);
  magnetometer.initialize(); 
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

void IMU::calibrateAndSetGyroSensorOffsets(int numErrorSamples = 2000)                                  
{                                           
  for (int sample; sample < numErrorSamples ; sample++)                                           
  {                                           
      updateMPUrawReadings();                                            
      gyroXsensorOffset += gyroXreading;                                             
      gyroYsensorOffset += gyroYreading;                                              
      gyroZsensorOffset += gyroZreading;                                              
      delay(3);                                                          
  }
  
  gyroXsensorOffset /= numErrorSamples;                                                  
  gyroYsensorOffset /= numErrorSamples;                                                  
  gyroZsensorOffset /= numErrorSamples;                                                   
}

float *IMU::getIMUstate()
{
  float *pitchAndRollAngles = calculateIMUAngles();                                                                                                                                             
  float yaw = getCurrentHeadingFromNorth();
  float altitudeInMeters = getCurrentAltitude(); 
  float tempFarenheit = getCurrentTemperatureC();  
  float values[] = {pitchAndRollAngles[0], pitchAndRollAngles[1], yaw, altitudeInMeters, tempFarenheit};
  return values;   
}

float *IMU::calculateIMUAngles()
{
    updateMPUrawReadings();
    //Accelerometer angle calculations
    float *gyroAngles = calculateGyroAngleEstimates();
    float *accelerometerAngles = calculateAccelerometerAngleEstimates();                                         

    float *filteredAngles = applyComplimentaryFilter(gyroAngles, accelerometerAngles);
    
    if(initialStartup)                                                                             
    {                                                 
        anglePitch = filteredAngles[0];
        angleRoll = filteredAngles[1];     
    }
    else
    {                                                                                         
        anglePitch = accelerometerAngles[0];                                                            
        angleRoll = accelerometerAngles[1];                                        
        initialStartup = true;                                                                 
    }    

    return filteredAngles;
}

void IMU::updateMPUrawReadings()
{
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  accReadingX = Wire.read()<<8|Wire.read();                                   //Add the low and high byte to the acc_x variable
  accReadingY = Wire.read()<<8|Wire.read();                                   //Add the low and high byte to the acc_y variable
  accReadingZ = Wire.read()<<8|Wire.read();                                   //Add the low and high byte to the acc_z variable
  int temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyroXreading = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the gyro_x variable
  gyroYreading = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the gyro_y variable
  gyroZreading = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the gyro_z variable 
}

float *IMU::calculateGyroAngleEstimates()
{
 float angles[] = {anglePitch, angleRoll};
 offsetGyroReadings();                                                  
 float *newGyroAngleEstimates = calculateGyroAngleReadings(angles);                                              
 coupleYawToPitchAndRollTransfers(newGyroAngleEstimates);
}

void IMU::offsetGyroReadings()
{
    gyroXreading -= gyroXsensorOffset;                                                                   
    gyroYreading -= gyroYsensorOffset;                                                
    gyroZreading -= gyroZsensorOffset; 
}

float *IMU::calculateGyroAngleReadings(float angles[2])
{
    angles[0] += convertGyroReadingToChangeInDegrees(gyroXreading);                                                       
    angles[1] += convertGyroReadingToChangeInDegrees(gyroYreading);  
    return angles;
}

float IMU::convertGyroReadingToChangeInDegrees(int gyroReading)
{
  float refreshRate = calcRefreshRate();
  float timeSinceLastUpdate = 1.0 / refreshRate;
  float readingToAngleChangeConversionRate = timeSinceLastUpdate * readingToDegreesConversion;
  return gyroReading * readingToAngleChangeConversionRate;
}

float *IMU::coupleYawToPitchAndRollTransfers(float angles[2])
{
    float yawChangeInDegrees = convertGyroReadingToChangeInDegrees(gyroZreading);
    float yawChangeInRadians = yawChangeInDegrees * (3.142 / 180.0);
    angles[0] += angleRoll * sin(yawChangeInRadians);                 //If the IMU is rotating, we must transfer the pitch and the roll between eachother in a sinosoidal wave form
    angles[1] -= anglePitch * sin(yawChangeInRadians);                 //this isn't the easiest concept to explain in comments, feel free to ask Corey
    return angles;
}

float *IMU::calculateAccelerometerAngleEstimates()
{
    float accelerometerVectorMag = sqrt(sq(accReadingX) + sq(accReadingY) + sq(accReadingZ));                                      //Calculate the total accelerometer vector magnitude
    
    float accelerometerPitchAngle = asin((float)accReadingY/accelerometerVectorMag)* 180 / 3.142;                                       //Calculate the pitch angle
    float accelerometerRollAngle = asin((float)accReadingX/accelerometerVectorMag)* -180 / 3.142;                                       //Calculate the roll angle

    float angles[2] = {accelerometerPitchAngle, accelerometerRollAngle};
    float *offsetAngles = offsetAccelerometerAngles(angles);
    return offsetAngles;
}

float *IMU::offsetAccelerometerAngles(float angles[2])
{
  angles[0] = accelerometerPitchOffset;
  angles[1] = accelerometerRollOffset;
  return angles;
}


//basically a weighted average of gyro and accelerometer readings
//result is essentially a bandpass filter that disregards random small spikes in accel readings
//and corrects long term drift of the gyro, sort of like an integral proportional controller but averaged
float *IMU::applyComplimentaryFilter(float gyroEstimates[2], float accelerometerEstimates[2])
{
    float filteredAngles[2];
    filteredAngles[0] = gyroEstimates[0] * gyroFilterWeight + accelerometerEstimates[0] * (1.0 - gyroFilterWeight);     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    filteredAngles[1] = gyroEstimates[1] * gyroFilterWeight + accelerometerEstimates[1] * (1.0 - gyroFilterWeight); 
    return filteredAngles;
}


float IMU::calcRefreshRate()
{
  int timeSinceLastFrame = millis() - lastIMUrefresh;
  float refreshRate = 1000.0 / timeSinceLastFrame; //1000 instead of one because we are in milliseconds going to seconds
  lastIMUrefresh = millis();
  return refreshRate;
}

float IMU::getCurrentAltitude()
{
  barometer.setControl(BMP085_MODE_PRESSURE_3);              //fast pressure measurment mode
  float currentAtmosphericPressure = barometer.getPressure();                                        
  return barometer.getAltitude(currentAtmosphericPressure);  
}

float IMU::getCurrentTemperatureC()
{
   barometer.setControl(BMP085_MODE_TEMPERATURE);
   return barometer.getTemperatureC();
}


float IMU::getCurrentHeadingFromNorth()
{
   int16_t mx, my, mz;
   magnetometer.getHeading(&mx, &my, &mz);              //getting magnetic field strength on each axis                                
   float heading = atan2(my, mx);                                               
   if(heading < 0) {heading += 2.0 * M_PI;}                               
   heading *= (180.0 / M_PI);
   return heading;                                             
}
