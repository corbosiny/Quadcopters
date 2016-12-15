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

//Declaring some global variables
int gyroX, gyroY, gyroZ;                                //holds the rate of change the quadcopter's angles detected by the MPU6050
long int accX, accY, accZ, accMag;                      //hold the acceleration values of their respective axsis and the magnitude
float anglePitch, angleRoll;                            //first the gyro calculated angles, then the IMU pitch and roll values
float angleRollAcc, anglePitchAcc;                      //calculated rotaion angles based off accelerometer
long gyroXoffset, gyroYoffet, gyroZoffset;              //used for calibration of the initial gyro reading offsets
boolean setGyroAngles = 0;                              //used to trigger initial angle readings, only is used on startup
int temperature;                                        //self explanatory bruh

int MPUaddr = 0x68;                                     //MPU address on the i2c bus, DO NOT CHANGE I WILL FIND YOU 
float accPitchOffset = 0, accRollOffset = 0;
float gyroContribution = .9996;                         //

void setup() 
{

  Wire.begin();                                                      
  Serial.begin(9600);                                                    //Use only for debugging
  pinMode(13, OUTPUT);                                                   //if this pin is in use take out this line
  
  setupMPU6050Registers();                                               //Setup the registers of the MPU-6050, look at datasheet for more info

  digitalWrite(13, HIGH);                                                //lets user know it is starting the callibration
  
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

  digitalWrite(13, LOW);                                               //lets the user know its is done calibrating
                                                                          //Reset the loop timer
}

void loop(){

  readMPU6050_data();                                                  //loads in all the raw readings from the MPU6050

  gyroX -= gyroXoffset;                                                //Subtracting the calibration offset from the raw gyro values
  gyroY -= gyroYoffset;                                                
  gyroZ -= gyroZoffset;                                                
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  anglePitch += gyroX * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angleRoll += gyroY * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) to convert to radians
  anglePitch += angleRoll * sin(gyroZ * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angleRoll -= anglePitch * sin(gyroZ * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  accMag = sqrt(sq(accX)+sq(accY)+sq(accZ));  //Calculate the total accelerometer vector
  
  //57.296 = 1 / (3.142 / 180) to convert to radians
  anglePitchAcc = asin((float)accY/accMag)* 57.296;       //Calculate the pitch angle
  angleRollAcc = asin((float)accX/accMag)* -57.296;       //Calculate the roll angle
  
  //set unit flat to determine these values
  //helps to determine initial angles better
  anglePitchAcc -= accPitchOffset;                                              //Accelerometer calibration value for pitch
  angleRollAcc -=  accRollOffset;                                               

  
  //below we usecomplimentary filter
  //basically a weighted average of gyro and accelerometer readings
  //result is essentially a bandpass filter that disregards random small spikes in accel readings
  //and corrects long term drift of the gyro, sort of like an integral proportional controller but averaged
  
  if(setGyroAngles)                                                                          //If the IMU is already started
  {                                                 
    anglePitch = anglePitch * gyroContribution + anglePitchAcc * (1 - gyroContribution);     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angleRoll = angleRoll * gyroContribution + angleRollAcc * (1 - gyroContribution);        
  }
  else
  {                                                                   //Only occurs first start
    anglePitch = anglePitchAcc;                                       //Set the initial angles to the accelerometer angles 
    angleRoll = angleRollAcc;                                        
    setGyroAngles = true;                                             //prevents any more triggering
  }
  
  temperature = temperature / 340.00 + 36.53;                         //from data sheet, converts temp reading to celcius
  
  Serial.print("Pitch: ");                                            //Debugging
  Serial.println(angle_pitch);
  Serial.print("Roll: ");
  Serial.println(angle_roll);
  Serial.print("Temperature: ")
  Serial.println(temperature);
  Serial.println("\n\n");
  
}

//--------------------------------------------------------------------------------------------------------
//-!!!!!!!!!!!!DONT EVER TOUCH THE CODE BELOW THIS EVER, I SWEAR THERE'LL BE HELL TO PAY!!!!!!!!!!!!!!!!!-
//-------------                                    From, Corey                           -----------------
//--------------------------------------------------------------------------------------------------------
void readMPU6050_data()                                                //loads in all the readings from the corresponding registers in the MPU
{                                                                      //all this comes from the data sheet 
                                                                       //ali, if you see this comment send me a text saying hi
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  accX = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  accY = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  accZ = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyroX = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyroY = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyroZ = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

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














