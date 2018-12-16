#ifndef IMU_h
#define IMU_h

#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"
#include <BMP085.h>
#include "HMC5883L.h"

// float accelerometerPitchOffset = 0, float accelerometerRollOffset = 0, float gyroFilterWeight = .9996 common initial values
class IMU
{

  friend class PID;
  public:
    IMU(float accelerometerPitchOffset, float accelerometerRollOffset, float gyroFilterWeight);                                                       

    void initializeIMUsensors();
    void initializeAngleEstimates();
    void setupMPU6050Registers();
    void calibrateAndSetGyroSensorOffsets(int numErrorSamples);
    float *getIMUstate();                                                      
    float *updateIMUAngles();
    void updateMPUrawReadings();
    float *calculateGyroAngleEstimates();
    float *calculateGyroAngleReadings(float angles[2]);
    void offsetGyroReadings();
    float convertGyroReadingToChangeInDegrees(int gyroReading);
    float *coupleYawToPitchAndRollTransfers(float angles[2]);
    float *calculateAccelerometerAngleEstimates();
    float *offsetAccelerometerAngles(float angles[2]);
    float *applyComplimentaryFilter(float gyroEstimates[2], float acceleromterEstimates[2]);
    float calcRefreshRate();                                                
    float getCurrentAltitude();
    float getCurrentTemperatureC();
    float getCurrentHeadingFromNorth();

    int gyroXreading, gyroYreading, gyroZreading;                                               
    long int accReadingX, accReadingY, accReadingZ;                                     
    float anglePitch, angleRoll;                                         
    float angleRollAcc, anglePitchAcc;                                   
    
  private:
    long gyroXsensorOffset, gyroYsensorOffset, gyroZsensorOffset;                                                                        
    long long int lastIMUrefresh = 0;
    
    float accelerometerPitchOffset = 0, accelerometerRollOffset = 0;       
    float gyroFilterWeight;                                                  //used for the complimentary filter, how much weight the gyro's measurments are given vs the accelerometer
    
    BMP085 barometer;                                                      
    HMC5883L magnetometer;                                                            
  
};

#endif

