/**************************************************************************************************
 * Kyneo data logger Example
 * 
 * This Example will show how to record data from the kyneo sensors on a mounted SD card
 * 
 * Created by GEKO Navsat S.L. for Kyneo V2.0 board
 * 
 * This example is free software, given under a GPL3 license
 * 
 * KYNEO is a product designed by GEKO Navsat S.L. in Europe http://www.gekonavsat.com
 * For further information, visit: http://kyneo.eu/
 * 
 *************************************************************************************************/

#include <KyneoBoard.h>                                                                                     // GekoNavsat libraries
#include <KyneoGNSS.h>
#include <KyneoUtils.h>
#include <FreeIMU.h>

#include <SPI.h>                                                                                            // Other libraries
#include <SD.h>

FreeIMU kyneoIMU;                                                                                           // Object definition
KyneoGNSS gnss;         
KyneoUtils util;   

float att[3];                                                                                               // Variable definition
float kyneoLat;
float kyneoLon;
float kyneoAlt;
char kyneoTime[10];

char file[12];

void setup()
{  
  Serial.begin(9600);                                                                                       // Initial setup
  delay(100);
  Serial.println("Setting up... ");
 
  pinMode(GP_LED_0, OUTPUT);
  pinMode(GP_LED_1, OUTPUT);
  
  Wire.begin();
  kyneoIMU.init();
  
  Serial1.begin(9600);
  delay(100);
  //set rate divider(GPGLL, GPRMC, GPVTG, GPGGA, GPGSA, GPGSV, PMTKCHN). 
  gnss.setNMEAOutput(    0,     0,     0,     1,     0,     0,       0);                                     // Period of each NMEA message
  //gnss.DisSBAS();
  //gnss.setFixRate(100); //milliseconds (100 - 10000)

  delay(100);
  
  uint8_t retries = 3;
  uint8_t filenum = 0;
  while(retries-- > 0){
    if (!SD.begin(SD_CS_PIN)) {                                                                              // SD card setup
      Serial.println("SD card failed, or not present");                                                   
    }
    else{
      Serial.println("SD card ready.");
      do{
        sprintf(file, "data%03u.txt", filenum++);      
      }
      while(SD.exists(file));   
      Serial.print("Data will be logged to file: ");
      Serial.println(file);
      break;
    }
  }
}

void loop()
{
  if(gnss.getLatLon(kyneoLat, kyneoLon) == 2){                                                               // Write the GNSS and IMU data if abailable
     
    gnss.gettime(kyneoTime);
    kyneoAlt = gnss.getalt();   
    kyneoIMU.getYawPitchRoll(att);

    File log = SD.open(file, FILE_WRITE);
    if(log == 0){
      Serial.println("Error logging data: log = 0");
    }else{
      
      digitalWrite(GP_LED_0, HIGH);
      
      log.print(att[0]);
      log.print(",");
      log.print(att[1]);
      log.print(",");
      log.print(att[2]);
      log.print(",");
      log.print(kyneoLat,6);
      log.print(",");
      log.print(kyneoLon,6);
      log.print(",");
      log.print(kyneoAlt);
      log.print(",");
      log.print(kyneoTime);
      log.println(";");
      
      log.close();
      Serial.println("IMU and GNSS data logged");
  
      digitalWrite(GP_LED_0, LOW);
    }
  }else{
                                                                                                             // Write the IMU data only if GNSS is not abailable
    kyneoIMU.getYawPitchRoll(att);

    File log = SD.open(file, FILE_WRITE);
    if(log == 0){
      Serial.println("Error logging data: log=0");
    }else{
      
      digitalWrite(GP_LED_1, HIGH);
          
      log.print(att[0]);
      log.print(",");
      log.print(att[1]);
      log.print(",");
      log.print(att[2]);
      log.println(",,,,;");
      
      log.close();
      Serial.println("IMU data logged, no GNSS data");
      
      digitalWrite(GP_LED_1, LOW);
    }
  }
  
  delay(100);
}

