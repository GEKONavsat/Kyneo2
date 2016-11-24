/**************************************************************************************************
 * Kyneo Test Bench.
 * 
 * Created by GEKO Navsat S.L.
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
 * With this sketch is possible to test the main features of Kyneo, using following commands:
 * 
 * A.- Euler Angles values
 * Q.- quaternion values
 * R.- Raw IMU sensors data
 * B.- Battery Voltage
 * L.- Blinking LEDs
 * G.- Location and time information
 * N.- Last received NMEA frame
 *************************************************************************************************/

#include <KyneoBoard.h>
#include <KyneoGNSS.h>
#include "KyneoUtils.h"
#include "FreeIMU.h"

char frame[100];
int gnss_idx = 0, newFrame = 0, options = 0, count = 0;
char cmd, a;
char day[10], hour[10];
float att[3];
float q[4];
float lat, lon;
int raw[9];
int headprint = 10;
      
FreeIMU kyneoIMU;           // FreeIMU object
KyneoGNSS gnss;             // Kyneo GNSS object
KyneoUtils util;            // Kyneo Utility object

void setup()
{
  Serial.begin(9600);
  Serial.print("Setting up... ");

  util.battInit();
  Wire.begin();
  kyneoIMU.init();
  if(KYNEO_VERSION == 1) pinMode(GP_LED_PIN, OUTPUT);
  
  gnss.init();
  //set rate divider(GPGLL, GPRMC, GPVTG, GPGGA, GPGSA, GPGSV, PMTKCHN). 
  gnss.setNMEAOutput(    0,     0,     0,     1,     0,     0,       0);          // Period of each NMEA message
                                                                          
  Serial.println("done!");
  Serial.println("--------------------------------------------");
  Serial.println("Welcome to the Kyneo testbench!");
}

void loop() {
  if(options == 0){
    Serial.println();
    Serial.println("Choose one of the following options:");
    Serial.println("    ");
    Serial.println("    A.- Euler Angles values");
    Serial.println("    Q.- quaternion values");
    Serial.println("    R.- Raw IMU sensors data");
    Serial.println("    B.- Battery Voltage");
    Serial.println("    L.- Blinking LEDs");
    Serial.println("    G.- Location and time information");
    Serial.println("    N.- Last received NMEA frame");
    Serial.println();
    
    while(!Serial.available());                  // Waits for user command
    while(Serial.available()){
      if(count++ == 0)cmd = Serial.read();
    }
    count = 0;
    options = 1;
  }else{
    if (cmd == 'a' || cmd == 'A'){
      kyneoIMU.getYawPitchRoll(att);
      Serial.println();
      Serial.print("Yaw: ");
      Serial.print(att[0]);
      Serial.print(" Pitch: ");
      Serial.print(att[1]);
      Serial.print(" Roll: ");
      Serial.print(att[2]);
      Serial.println("");
    }else if (cmd == 'q' || cmd == 'Q'){
      Serial.println();
      Serial.println("q0\tq1\tq2\tq3"); 
      kyneoIMU.getQ(q);
      Serial.print(q[0]);
      Serial.print("\t");
      Serial.print(q[1]);
      Serial.print("\t");
      Serial.print(q[2]);
      Serial.print("\t");
      Serial.println(q[3]);
    }else if (cmd == 'r' || cmd == 'R'){
        kyneoIMU.getRawValues(raw);
        Serial.println("acc_x\tacc_y\tacc_z\tgyr_x\tgyr_y\tgyr_z\tmag_x\tmag_y\tmag_z\tTemp\tAlt"); 
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
        Serial.print(kyneoIMU.baro.getTemperature(MS561101BA_OSR_4096), 1);   //temp
        Serial.print("\t");
        Serial.println(kyneoIMU.getBaroAlt(), 2);   //pres
    }else if (cmd == 'b' || cmd == 'B'){
        Serial.print("Battery level (mV): ");
        Serial.println(util.battLevel());
    }else if (cmd == 'l' || cmd == 'L'){
      if(KYNEO_VERSION == 1){
        Serial.println("Blinking LED!!");
        digitalWrite(GP_LED_PIN, HIGH);   // LED ON.
        delay(500);
        digitalWrite(GP_LED_PIN, LOW);    // LED OFF.
        delay(500);
      }
    }else if (cmd == 'g' || cmd == 'G'){
      Serial.println("Location and time information: ");
        if(gnss.getLatLon(lat, lon) == 2){
          gnss.getdate(day);
          gnss.gettime(hour);
          Serial.print("    Fecha:    ");
          Serial.println(day);
          Serial.print("    Hora:     ");
          Serial.println(hour);
          Serial.print("    Latitud:  ");
          Serial.println(gnss.getlat());
          Serial.print("    Longitud: ");
          Serial.println(gnss.getlon());
          Serial.print("    Altitud:  ");
          Serial.println(gnss.getalt());
        }else{
          Serial.println("No GNSS information read...");
        }
    }else if (cmd == 'n' || cmd == 'N'){
      Serial.println("Last NMEA frame received:");
      if( gnss.getSingleNMEA(frame, 100)!=0 ) Serial.println(frame);
    }else{
      Serial.println("Command not found");
    }
    while(Serial.available()) a = Serial.read();                        // Serial buffer flushing
    options = 0;
  }
}

