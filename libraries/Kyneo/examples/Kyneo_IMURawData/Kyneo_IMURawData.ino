/**************************************************************************************************
 * Kyneo Basic IMU Sensors Raw Data Measurement.
 * 
 * Created on 11 Nov 2016 by GEKO Navsat S.L.
 * 
 * This example is free software, given under a GPL3 license.
 * 
 * KYNEO is an Arduino-compliant board which includes Movement & Location Sensors and a GNSS device. 
 * All these sensors' data can be logged into an micro-SD or, if a XBee compatible RF module is 
 * attached, they can be wirelessly shared among other devices within the network.
 * 
 * KYNEO is a product designed by GEKO Navsat S.L. 
 * http://www.gekonavsat.com
 * 
 * This sketch prints sensors data through serial and shows some configuration commands. It uses 
 * FreeIMU library (by Fabio Varesano), which has been adapted to support Kyneo board.
 * Kyneo MARG includes:
 *  - A 3-axis accelerometer + gyroscope (MPU6050): 16-bit ADC converter, auxiliary and primary I2C
 *    interfaces, default full scale config is +-2g (1 g = 9.806 m/s^2) and +- 250deg/s.
 *  - A magnetometer (HMC5883): 12-bit ADC converter, I2C interface (connected to MPU6050), default
 *    full scale gain in FreeIMU lib is +-1.2 Gauss (1 G = 10^-4 T).
 *  - A barometric altimeter (MS561101BA): 24-bit ADC for temperature and pressure sensors, I2C
 *    interface. Data range from -40 to 85ºC and from 100 to 1200 mbar. 
 *************************************************************************************************/

#include <KyneoBoard.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <DebugUtils.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <HMC58X3.h>
#include <MS561101BA.h>
#include <FreeIMU.h>

int raw[11];
int counter = 9;
FreeIMU kyneoMARG = FreeIMU();

void setup()
{
  Serial.begin(9600);
  FreeIMUinit();
  Serial.println("Setup done\n");
}

void loop()
{
  counter++;
  if(counter == 10){              // Prints column titles every 10 lines
    counter = 0;
    Serial.println("acc_x\tacc_y\tacc_z\tgyr_x\tgyr_y\tgyr_z\tmag_x\tmag_y\tmag_z\tTemp\tAlt");  
  }
  
  kyneoMARG.getRawValues(raw);
  // Print measures: 
  //  accelerometer xyz - in g (1 g = 9.806 m/s2), dividing by +- 16g scale factor. 
  //  gyroscope xyz - in deg/s, dividing by +-2000 deg/s scale factor. 
  //  magnetometer xyz - in milliGauss, dividing by +-1.2 Gauss scale factor.
  //  temperature (ºC) and altitude (m) 
  Serial.print((float)raw[0]/2048, 2);    // acc_x
  Serial.print("\t");
  Serial.print((float)raw[1]/2048, 2);    // acc_y
  Serial.print("\t");
  Serial.print((float)raw[2]/2048, 2);    // acc_z
  Serial.print("\t");
  Serial.print((float)raw[3]/16.384, 2);  //gyr_x
  Serial.print("\t");
  Serial.print((float)raw[4]/16.384, 2);  //gyr_y
  Serial.print("\t");
  Serial.print((float)raw[5]/16.384, 2);  //gyr_z
  Serial.print("\t");
  Serial.print((float)raw[6]/1.707, 1);   //mag_x
  Serial.print("\t");
  Serial.print((float)raw[7]/1.707, 1);   //mag_y
  Serial.print("\t");
  Serial.print((float)raw[8]/1.707, 1);   //mag_z
  Serial.print("\t");
  Serial.print(kyneoMARG.baro.getTemperature(MS561101BA_OSR_4096), 1);   //temp
  Serial.print("\t");
  Serial.println(kyneoMARG.getBaroAlt(), 2);   //pres
  delay(1000);
}

void FreeIMUinit(){
  Wire.begin();
  delay(5);
  kyneoMARG.init(true); // Init sensors (default FS settings). Optionally, "true" sets I2C fast mode. 
  delay(5);

  // User FS range: sets the new Full Scale settings for accelerometer and gyroscope.
  do{
    kyneoMARG.accgyro.setFullScaleAccelRange(MPU60X0_ACCEL_FS_16);           // +-16g
    delay(5);
  }
  while(kyneoMARG.accgyro.getFullScaleAccelRange() != MPU60X0_ACCEL_FS_16);  // +-16g
  do{
    kyneoMARG.accgyro.setFullScaleGyroRange(MPU60X0_GYRO_FS_2000);           // +-2000 deg/s        
    delay(5);
  }
  while(kyneoMARG.accgyro.getFullScaleGyroRange() != MPU60X0_GYRO_FS_2000);  // +-2000 deg/s
}

