//  This is an example for the BMP085 Barometric Pressure and Temp Sensor
// Connect VCC of the BMP085 sensor to 3.3V
// Connect GND to Ground
// Connect SCL to I2C clock (on Arduino Nano/Uno/Duemilanove/etc thats Analog 5
// Connect SDA to I2C data (on Arduino Nano/Uno/Duemilanove/etc thats Analog 4

#include <Wire.h>
#include <BMP085.h>

BMP085 bmp;
  
void setup() {
  Serial.begin(9600);
  bmp.initialize();
    bmp.setControl(BMP085_MODE_PRESSURE_1);
}
  
void loop() {

    
    Serial.print(" Pressure = ");
    Serial.print(bmp.getPressure());
    Serial.print(" Pa, ");
    Serial.print(" Altitude = ");
    Serial.print(bmp.getAltitude(101500));
    Serial.print(" meters" );
    Serial.println();
    delay(400);
}
