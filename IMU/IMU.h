#ifndef IMU_h
#define IMU_h

#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"
#include <BMP085.h>
#include "HMC5883L.h"

class IMU
{

  friend class PID;

  public:
    IMU(long int seaLevelPressure, float gyroContribution = .9996, float accPitchOffset = 0, float accRollOffset = 0, int refreshRate = 250);                                                                   //constructor

    float *readIMUData();                                                    //Will run through and call all of the below functions to update all the IMU data
    void readMPUDataRaw();                                                   //Handles getting all the raw data from the MPU6050 and prepares it for calculations
    void readMPUData();                                                      //Converts raw MPU6050 data in readable/useable forms that actually have meaning
    void readBMPData();                                                      //Reads in barometer(altitude/airpressure) data
    void readMCLData();                                                      //Reads in directional data(bearing in relation to North)
    void setupMPU6050Registers();
    void calibrateGyro();

    int gyroX, gyroY, gyroZ;                                                 //holds the rate of change the quadcopter's angles detected by the MPU6050
    long int accX, accY, accZ, accMag;                                       //hold the acceleration values of their respective axsis and the magnitude
    float anglePitch, angleRoll;                                             //first the gyro calculated angles, then the IMU pitch and roll values
    float angleRollAcc, anglePitchAcc;                                       //calculated rotaion angles based off accelerometer
    float heading;                                                           //will store your angle in relation to the north pole, 0 = North
    long int alt;                                                            //holds the current altitude
    
  private:
    long gyroXoffset, gyroYoffset, gyroZoffset;                              //used for calibration of the initial gyro reading offsets
    boolean setGyroAngles = 0;                                               //used to trigger initial angle readings, only is used on startup
    int temperature;                                                         //self explanatory bruh
    
    float accPitchOffset = 0, accRollOffset = 0;                             //offsets for the accelerometer values, laying it flat is the best way to calculate these
    float gyroContribution;                                                  //used for the complimentary filter, how much weight the gyro's measurments are given vs the accelerometer
    int refreshRate;                                                         //Used as the dt in the pseudo integral calculations
    
    BMP085 bmp;                                                              //The barometer object
    int pressure;                                                            //holds the air pressure readings
    long int seaLevelPress;                                                  //holds the local air pressure at sea level(used for altitude), must be entered
    
    HMC5883L mag;                                                            //The magentometer object
    int16_t mx, my, mz;                                                      //holds magentic field values on each axis
    
  
  
};

#endif

