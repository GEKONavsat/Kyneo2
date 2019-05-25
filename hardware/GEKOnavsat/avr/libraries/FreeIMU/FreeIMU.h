/*
FreeIMU.h - A libre and easy to use orientation sensing library for Arduino
Copyright (C) 2011 Fabio Varesano <fabio at varesano dot net>

Development of this code has been supported by the Department of Computer Science,
Universita' degli Studi di Torino, Italy within the Piemonte Project
http://www.piemonte.di.unito.it/


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

//INFO-GEKONavsat: Modified to include KYNEO v1 board support.

#ifndef FreeIMU_h
#define FreeIMU_h

// Uncomment the appropriated version of FreeIMU you are using
//#define FREEIMU_v01
//#define FREEIMU_v02
//#define FREEIMU_v03
//#define FREEIMU_v035
//#define FREEIMU_v035_MS
//#define FREEIMU_v035_BMP
//#define FREEIMU_v04

// 3rd party boards. Please consider donating or buying a FreeIMU board to support this library development.
//#define SEN_10121 //IMU Digital Combo Board - 6 Degrees of Freedom ITG3200/ADXL345 SEN-10121 http://www.sparkfun.com/products/10121
//#define SEN_10736 //9 Degrees of Freedom - Razor IMU SEN-10736 http://www.sparkfun.com/products/10736
//#define SEN_10724 //9 Degrees of Freedom - Sensor Stick SEN-10724 http://www.sparkfun.com/products/10724
//#define SEN_10183 //9 Degrees of Freedom - Sensor Stick  SEN-10183 http://www.sparkfun.com/products/10183
//#define ARDUIMU_v3 //  DIYDrones ArduIMU+ V3 http://store.diydrones.com/ArduIMU_V3_p/kt-arduimu-30.htm or https://www.sparkfun.com/products/11055
//#define GEN_MPU6050 // Generic MPU6050 breakout board. Compatible with GY-521, SEN-11028 and other MPU6050 wich have the MPU6050 AD0 pin connected to GND.
#define KYNEO_v1	// GEKO Navsat Kineo v1.0 board


//#define DISABLE_MAGN // Uncomment this line to disable the magnetometer in the sensor fusion algorithm

// *** No configuration needed below this line ***

#define FREEIMU_LIB_VERSION "DEV"

#define FREEIMU_DEVELOPER "Fabio Varesano - varesano.net"

#if F_CPU == 16000000L
  #define FREEIMU_FREQ "16 MHz"
#elif F_CPU == 12000000L
  #define FREEIMU_FREQ "12 MHz"
#elif F_CPU == 8000000L
  #define FREEIMU_FREQ "8 MHz"
#endif


// board IDs
#if defined(FREEIMU_v01)
  #define FREEIMU_ID "FreeIMU v0.1"
#elif defined(FREEIMU_v02)
  #define FREEIMU_ID "FreeIMU v0.2"
#elif defined(FREEIMU_v03)
  #define FREEIMU_ID "FreeIMU v0.3"
#elif defined(FREEIMU_v035)
  #define FREEIMU_ID "FreeIMU v0.3.5"
#elif defined(FREEIMU_v035_BMP)
  #define FREEIMU_ID "FreeIMU v0.3.5_BMP"
#elif defined(FREEIMU_v035_MS)
  #define FREEIMU_ID "FreeIMU v0.3.5_MS"
#elif defined(FREEIMU_v04)
  #define FREEIMU_ID "FreeIMU v0.4"
#elif defined(SEN_10121)
  #define FREEIMU_ID "SparkFun 10121"
#elif defined(SEN_10736)
  #define FREEIMU_ID "SparkFun 10736"
#elif defined(SEN_10724)
  #define FREEIMU_ID "SparkFun 10724"
#elif defined(SEN_10183)
  #define FREEIMU_ID "SparkFun 10183"
#elif defined(ARDUIMU_v3)
  #define FREEIMU_ID "DIY Drones ArduIMU+ V3"
#elif defined(KYNEO_v1)
  #define FREEIMU_ID "GEKO NavSat Kineo v1.0"
#endif

// Board Sensors
#define HAS_ITG3200() 	(defined(FREEIMU_v01)	|| defined(FREEIMU_v02) 	|| defined(FREEIMU_v03) 	|| \
						defined(FREEIMU_v035)	|| defined(FREEIMU_v035_MS)	|| defined(FREEIMU_v035_BMP)|| \
						defined(SEN_10121) 		|| defined(SEN_10736) 		|| defined(SEN_10724) 		|| \
						defined(SEN_10183))
#define HAS_ADXL345()	(defined(FREEIMU_v01)	|| defined(FREEIMU_v02) 	|| defined(FREEIMU_v03) 	|| \
						defined(SEN_10121) 		|| defined(SEN_10736) 		|| defined(SEN_10724) 		|| \
						defined(SEN_10183))
#define HAS_BMA180() 	(defined(FREEIMU_v035) 	|| defined(FREEIMU_v035_MS)	|| defined(FREEIMU_v035_BMP))
#define HAS_MPU6050() 	(defined(KYNEO_v1)		|| defined(FREEIMU_v04) 	|| defined(GEN_MPU6050))
#define HAS_MS5611() 	(defined(KYNEO_v1)		|| defined(FREEIMU_v035_MS)	|| defined(FREEIMU_v04))	
#define HAS_HMC5883L() 	(defined(KYNEO_v1)		|| defined(FREEIMU_v01) 	|| defined(FREEIMU_v02) 	|| \
						defined(FREEIMU_v03) 	|| defined(FREEIMU_v035) 	|| defined(FREEIMU_v035_BMP)|| \
						defined(FREEIMU_v035_MS)|| defined(FREEIMU_v04) 	|| defined(SEN_10736) 		|| \
						defined(SEN_10724) 		|| defined(SEN_10183)  		|| defined(ARDUIMU_v3))
#define HAS_MPU6000() 	(defined(ARDUIMU_v3))

#define IS_6DOM() 		(defined(SEN_10121) 	|| defined(GEN_MPU6050))
#define IS_9DOM() 		(defined(KYNEO_v1)		|| defined(FREEIMU_v01) 	|| defined(FREEIMU_v02) 	|| \
						defined(FREEIMU_v03) 	|| defined(FREEIMU_v035) 	|| defined(FREEIMU_v035_BMP)|| \
						defined(FREEIMU_v035_MS)|| defined(FREEIMU_v04) 	|| defined(SEN_10736) 		|| \
						defined(SEN_10724) 		|| defined(SEN_10183) 		|| defined(ARDUIMU_v3))
						
#define HAS_AXIS_ALIGNED() 	(defined(KYNEO_v1)	|| defined (FREEIMU_v01) 	|| defined(FREEIMU_v02) 	|| \
							defined(FREEIMU_v03)|| defined(FREEIMU_v035) 	|| defined(FREEIMU_v035_MS) || \
							defined(FREEIMU_v035_BMP)|| defined(FREEIMU_v04)|| defined(SEN_10121) 		|| \
							defined(SEN_10736))
#include <Wire.h>
#include "Arduino.h"
#include "calibration.h"

#ifndef CALIBRATION_H
#include <EEPROM.h>
#endif

#define FREEIMU_EEPROM_BASE 0x0A
#define FREEIMU_EEPROM_SIGNATURE 0x19

//#if FREEIMU_VER <= 3
#if HAS_ADXL345()
  #include <ADXL345.h>
  // default I2C 7-bit addresses of the sensors
  #define FIMU_ACC_ADDR ADXL345_ADDR_ALT_LOW // SDO connected to GND
  //#define FIMU_ADXL345_DEF_ADDR ADXL345_ADDR_ALT_HIGH // SDO connected to GND
#elif HAS_BMA180()
  #include <bma180.h>
  #define FIMU_ACC_ADDR BMA180_ADDRESS_SDO_LOW 
  //#define FIMU_ACC_ADDR BMA180_ADDRESS_SDO_HIGH
//#elif FREEIMU_VER == 7
#elif HAS_MPU6050()
  #include <Wire.h>
  #include "I2Cdev.h"
  #include "MPU60X0.h"
  #define FIMU_ACCGYRO_ADDR MPU60X0_DEFAULT_ADDRESS
#elif HAS_MPU6000()
  #include <SPI.h>
  #include "I2Cdev.h"
  #include "MPU60X0.h"
  #define FIMU_ACCGYRO_ADDR MPU60X0_DEFAULT_SS_PIN
#endif


#if HAS_MS5611()
  #include <MS561101BA.h>
  #define FIMU_BARO_ADDR MS561101BA_ADDR_CSB_LOW
  //#define FIMU_BARO_ADDR MS561101BA_ADDR_CSB_HIGH
#endif

#if HAS_ITG3200()
  #include <ITG3200.h>
#endif

#if HAS_HMC5883L()
  #include <HMC58X3.h>
#endif


#define FIMU_BMA180_DEF_ADDR BMA180_ADDRESS_SDO_LOW
#define FIMU_ITG3200_DEF_ADDR ITG3200_ADDR_AD0_LOW // AD0 connected to GND
// HMC5843 address is fixed so don't bother to define it


#define twoKpDef  (2.0f * 0.75f)//0.75f)	//(2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.1625f)//0.1625f) // 2 * integral gain

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
/////////////////////////////////---------------------------------------------
#define betaDef		0.075f		// 2 * proportional gain
/////////////////////////////////---------------------------------------------

/////////////////////////////////---------------------------------------------

/// Definición de los factores de escala
/// Ojo, solo hecho para la MPU6050. No se tiene en cuenta que en otras placas puedan ser otros valores diferentes
// Para la calibración, seleccionar los FS +-2g para obtener resultados lo mas precisos posibles. El FS del giroscopo no importa. El del magnetometro (la ganancia) se deja fijo según lo que ya hay.

//#define FS_ACC 2	// +-2g
//#define FS_ACC 4	// +-4g
//#define FS_ACC 8	// +-8g
#define FS_ACC 16	// +-16g

//#define FS_GYRO 250	//+-250º/s
//#define FS_GYRO 500	//+-500º/s
//#define FS_GYRO 1000	//+-1000º/s
#define FS_GYRO 2000	//+-2000º/s


//MPU60X0_ACCEL_FS_2          0x00
//MPU60X0_ACCEL_FS_4          0x01
//MPU60X0_ACCEL_FS_8          0x02
//MPU60X0_ACCEL_FS_16         0x03
#if (FS_ACC == 2)
#define MPU60X0_ACCEL_FS	0x00
#define MPU60X0_1G_LSB	16384
#elif (FS_ACC == 4)
#define MPU60X0_ACCEL_FS	0x01
#define MPU60X0_1G_LSB	8192
#elif (FS_ACC == 8)
#define MPU60X0_ACCEL_FS	0x02
#define MPU60X0_1G_LSB	4096
#elif (FS_ACC == 16)
#define MPU60X0_ACCEL_FS	0x03
#define MPU60X0_1G_LSB	2048
#else
#error "Accelerometer FS not valid"
#endif

//Definimos el FS de +-2g que usaremos en la conversión de los parametros de calibración
#define MPU60X0_ACCEL_1G_LSB_2G	16384

/// Los LSB del gyroscopo aqui definidos no se usan. se ponen por si se necesitan
//MPU60X0_GYRO_FS_250         0x00
//MPU60X0_GYRO_FS_500         0x01
//MPU60X0_GYRO_FS_1000        0x02
//MPU60X0_GYRO_FS_2000        0x03
#if (FS_GYRO == 250)
#define MPU60X0_GYRO_FS	0x00
#define MPU60X0_1DEGREE_LSB	131.0f
#elif (FS_GYRO == 500)
#define MPU60X0_GYRO_FS	0x01
#define MPU60X0_1DEGREE_LSB	65.5f
#elif (FS_GYRO == 1000)
#define MPU60X0_GYRO_FS	0x02
#define MPU60X0_1DEGREE_LSB	32.8f
#elif (FS_GYRO == 2000)
#define MPU60X0_GYRO_FS 0x03
#define MPU60X0_1DEGREE_LSB	16.4f
#else
#error "Gyroscope FS not valid"
#endif

/////////////////////////////////---------------------------------------------

//#define MAGN_DISABLED //////////////*************** Añadido para probar placa Kyneo v1 en la que no funciona el magnetometro

class FreeIMU
{
  public:
    FreeIMU();
	
	//Para compensación de temperatura
	  // TC
	float m[3], b[3];
	int reft;
	byte state;
	
	//Para compensación de temperatura
	void tc_param(float mx,float my,float mz,float bx, float by,float bz, int t); // sets parameters for TC  
	void tc_bias();
	
    void init();
    void init(bool fastmode);
    #if HAS_ITG3200()
    void init(int acc_addr, int gyro_addr, bool fastmode);
    #else
    void init(int accgyro_addr, bool fastmode);
    #endif
    #ifndef CALIBRATION_H
    void calLoad();
    #endif
    void zeroGyro();
    void getRawValues(int * raw_values);
    void getValues(float * values);
    void getQ(float * q);
    void getEuler(float * angles);
    void getYawPitchRoll(float * ypr);
    void getEulerRad(float * angles);
    void getYawPitchRollRad(float * ypr);
    #if HAS_MS5611()
      float getBaroAlt();
      float getBaroAlt(float sea_press);
      //float getEstimatedAlt();
      //float getEstimatedAlt(float sea_press);
    #endif
    void gravityCompensateAcc(float * acc, float * q);
    void accCompensateGravity(float * acc, float * q);			//XXX-GEKONavsat: lib bug? added declaration
	
    // we make them public so that users can interact directly with device classes
    #if HAS_ADXL345()
      ADXL345 acc;
    #elif HAS_BMA180()
      BMA180 acc;
    #endif
    
    #if HAS_HMC5883L()
      HMC58X3 magn;
    #endif
    
    
    #if HAS_ITG3200()
      ITG3200 gyro;
    #elif HAS_MPU6050()
      MPU60X0 accgyro; 
    #elif HAS_MPU6000()
      MPU60X0 accgyro; 
    #endif
      
      
    #if HAS_MS5611()
      MS561101BA baro;
    #endif
    
	
    int* raw_acc, raw_gyro, raw_magn;
    // calibration parameters
    int16_t gyro_off_x, gyro_off_y, gyro_off_z;
    int16_t acc_off_x, acc_off_y, acc_off_z, magn_off_x, magn_off_y, magn_off_z;
    float acc_scale_x, acc_scale_y, acc_scale_z, magn_scale_x, magn_scale_y, magn_scale_z;
    
	/// parte de demostración del no uso de calibración
	void getUncalibValues(float * uncalibValues);
	void getUncalibratedQ(float * q);
	//
	
  private:
    #if IS_9DOM()
    void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    #elif IS_6DOM()
    void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);
    #endif
    //float q0, q1, q2, q3; // quaternion elements representing the estimated orientation
    float iq0, iq1, iq2, iq3;
    float exInt, eyInt, ezInt;  // scaled integral error
    volatile float twoKp;      // 2 * proportional gain (Kp)
    volatile float twoKi;      // 2 * integral gain (Ki)
    volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
	// Para uncalibrated
	volatile float qU0, qU1, qU2, qU3; // quaternion of sensor frame relative to auxiliary frame
	//
    volatile float integralFBx,  integralFBy, integralFBz;
    unsigned long lastUpdate, now; // sample period expressed in milliseconds
	unsigned long lastUpdateUncalib, nowUncalib;
    float sampleFreq; // half the sample period expressed in seconds
	float sampleFreqUncalib;
	
	/////////////////////////
	volatile float beta;
	void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
	
	void MadgwickAHRSupdateUncalibrated(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	/////////////////////////
	
	/////////////////////////
	void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

	/////////////////////////
    
};

float invSqrt(float number);
void arr3_rad_to_deg(float * arr);



#endif // FreeIMU_h

