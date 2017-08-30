/**************************************************************************************************
 * Kyneo IMU raw data example
 * 
 * This sketch prints sensors data through serial. It uses FreeIMU library (by Fabio Varesano), 
 * which has been adapted to support Kyneo board.
 * Kyneo MARG includes:
 *  - A 3-axis accelerometer + gyroscope (MPU6050): 16-bit ADC converter, auxiliary and primary I2C
 *    interfaces,
 *  - A magnetometer (HMC5883): 12-bit ADC converter, I2C interface (connected to MPU6050), 
 *  - A barometric altimeter (MS561101BA): 24-bit ADC for temperature and pressure sensors, I2C
 *    interface.
 * 
 * 
 * Created by GEKO Navsat S.L. for Kyneo V2.0 board
 * 
 * This example is free software, given under a GPL3 license
 * 
 * KYNEO is a product designed by GEKO Navsat S.L. in Europe http://www.gekonavsat.com
 * For further information, visit: http://kyneo.eu/
 * 
 *************************************************************************************************/
#include <KyneoBoard.h>                                 //GekoNavsat libraries
#include <FreeIMU.h>

FreeIMU kyneoIMU;                                       //Object definition

int LoopRate = 10;  //milliseconds                      //Variable definition
int raw[11];
int counter = 9;


void setup()
{
  Serial.begin(9600);
  
  Wire.begin();
  kyneoIMU.init();
  
  Serial.println("Setup done\n");
}

void loop()
{
  counter++;
  if(counter == 10){                                    // Prints column titles every 10 lines
    counter = 0;  
    Serial.println("acc_x\tacc_y\tacc_z\tgyr_x\tgyr_y\tgyr_z\tmag_x\tmag_y\tmag_z\tTemp\tAlt");  
  }
                                                                            
  kyneoIMU.getRawValues(raw);                                               
  Serial.print((float)raw[0]/2048, 2);                                    // acc_x
  Serial.print("\t");
  Serial.print((float)raw[1]/2048, 2);                                    // acc_y
  Serial.print("\t");
  Serial.print((float)raw[2]/2048, 2);                                    // acc_z
  Serial.print("\t");
  Serial.print((float)raw[3]/16.384, 2);                                  // gyr_x
  Serial.print("\t");
  Serial.print((float)raw[4]/16.384, 2);                                  // gyr_y
  Serial.print("\t");
  Serial.print((float)raw[5]/16.384, 2);                                  // gyr_z
  Serial.print("\t");
  Serial.print((float)raw[6]/1.707, 1);                                   // mag_x
  Serial.print("\t");
  Serial.print((float)raw[7]/1.707, 1);                                   // mag_y
  Serial.print("\t");
  Serial.print((float)raw[8]/1.707, 1);                                   // mag_z
  Serial.print("\t");
  Serial.print(kyneoIMU.baro.getTemperature(MS561101BA_OSR_4096), 1);     // temp
  Serial.print("\t");
  Serial.println(kyneoIMU.getBaroAlt(), 2);                               // pres
  
  delay(LoopRate);
}
/*********************************************************************************
// Printed measures: 
//  accelerometer xyz - in g (1 g = 9.806 m/s2), dividing by +- 16g scale factor. 
//  gyroscope xyz - in deg/s, dividing by +-2000 deg/s scale factor. 
//  magnetometer xyz - in milliGauss, dividing by +-1.2 Gauss scale factor.
//  temperature (ºC) and altitude (m) 
**********************************************************************************/


