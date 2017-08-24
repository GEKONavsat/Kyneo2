/**************************************************************************************************
 * Kineo Datalogger Example.
 * 
 * Created on 2017 by GEKO Navsat S.L.
 * 
 * This example is free software, given under a GPL3 license.
 * 
 * KYNEO is an Arduino-compliant board which includes Movement & Location Sensors and a GNSS device. 
 * All these sensors' data can be logged into an micro-SD or, if a XBee compatible RF module is 
 * attached, they can be wirelessly shared among other devices within the network.
 * 
 * Kyneo is a product designed by GEKO Navsat S.L. 
 * http://www.gekonavsat.com
 * 
 * This simple sketch uses Kineo board to make a log of EGI (Embeded GNSS INS) data, that is 
 * composed by GNSS position (lat, lon, alt) and inertial attitude (Euler angles). The data
 * is stored in a .txt file.
 * 
 *************************************************************************************************/
#include <KyneoBoard.h>
#include <KyneoGNSS.h>
#include "KyneoUtils.h"
#include "FreeIMU.h"

#include <Wire.h>
#include <SPI.h>
#include <SD.h>

FreeIMU kyneoIMU;  // FreeIMU object
KyneoGNSS gnss;    // Kyneo GNSS object      
KyneoUtils util;   // Kyneo parser object

float att[3];
float kyneoLat = 0.0;
float kyneoLon = 0.0;
float kyneoAlt = 0.0;
char kyneoTime[10];
int gnss_idx = 0;

char file[12];
char frame[100];

void setup()
{  
  Serial.begin(9600);
  Serial.println("Setting up... ");

  Wire.begin();
  kyneoIMU.init();
  pinMode(GP_LED_0, OUTPUT);
  pinMode(GP_LED_1, OUTPUT);
  
  Serial1.begin(9600);
  //set rate divider(GPGLL, GPRMC, GPVTG, GPGGA, GPGSA, GPGSV, PMTKCHN). 
  gnss.setNMEAOutput(    0,     0,     0,     1,     0,     0,       0);  //Period of each NMEA message
  
  uint8_t retries = 3;
  uint8_t filenum = 0;
  while(retries-- > 0){
    if (!SD.begin(SD_CS_PIN)) {
      Serial.println("SD card failed, or not present");
    }else{
      Serial.println("SD card ready.");
      do{
        sprintf(file, "data%03u.csv", filenum++);      
      }while(SD.exists(file));   
      Serial.print("Data will be logged to file: ");
      Serial.println(file);
      break;
    }
  }
  delay(500);
}

void loop()
{
  gnss_idx = gnss.getLatLon(kyneoLat, kyneoLon);
  
  if(gnss_idx == 2){
    digitalWrite(GP_LED_0, HIGH);   // LED0 ON. 
    kyneoIMU.getYawPitchRoll(att);
    kyneoAlt = gnss.getalt();
    gnss.gettime(kyneoTime);
    
    File log = SD.open(file, FILE_WRITE);
    if(log){
      delay(50);
      log.print(att[0]);        //Yaw
      log.print(",");
      log.print(att[1]);        //Pitch
      log.print(",");
      log.print(att[2]);        //Roll
      log.print(",");
      log.print(kyneoLat,6);    //Latitude
      log.print(",");
      log.print(kyneoLon,6);    //Longitude
      log.print(",");
      log.print(kyneoAlt);      //Altitude
      log.print(",");
      log.println(kyneoTime);   //Time
      delay(50);
      log.close();
      Serial.println("data logged");
      
    }else{
      Serial.println("Error logging data: log=0");
    }
    delay(100);
    digitalWrite(GP_LED_0, LOW);    // LED0 OFF.

  }else{
    digitalWrite(GP_LED_1, HIGH);   // LED1 ON.
    Serial.println("no data to log");
    delay(200);
    digitalWrite(GP_LED_1, LOW);    // LED1 OFF.
  }
}

