/**************************************************************************************************
 * Kyneo GNSS Location Data Example.
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
 * This sketch makes use of KyneoGNSS library, which internally uses NMEAparser library as well.
 * 
 * This code example does the following:
 * 1) Initialise connection with the GNSS device and configures the NMEA frames that are going to be
 * sent from it:
 * - Serial1 Config: 9600 bauds
 * - NMEA output: 1Hz fix rate. GPGGA, GPRMC: (rate/1)
 * 
 * Note: the GNSS device goes back to default settings removing the/running out of battery.
 * 
 * 2) Sends the basic location information (time, date, latitude, longitude, altitude) via Serial port
 * (Kyneo USB-Serial interface).
 * 
 ******************************************************************************************************/
#include <KyneoBoard.h>
#include <KyneoGNSS.h>

KyneoGNSS gnss;
float lat = 0, lon = 0;     // longitud and latitude init
char hour[10], day[10];

void setup()
{
  Serial.begin(9600);       // Choose your preferred baudrate here.
  Serial1.begin(9600);      // Default baudrate to connect with GNSS module

  //set rate divider(GPGLL, GPRMC, GPVTG, GPGGA, GPGSA, GPGSV, PMTKCHN). 
  gnss.setNMEAOutput(    0,     1,     0,     1,     0,     0,       0);    //GPRMC 1Hz, GPGGA 1Hz
  Serial.println("GNSS Setup completed.");
  Serial.println();
}

void loop()
{
  if(gnss.getLatLon(lat, lon) == 2){
    gnss.getdate(day);
    gnss.gettime(hour);
    
    Serial.println("--------------------");
    Serial.print("Fecha:    ");
    Serial.println(day);
    Serial.print("Hora:     ");
    Serial.println(hour);
    Serial.print("Latitud:  ");
    Serial.println(gnss.getlat());
    Serial.print("Longitud: ");
    Serial.println(gnss.getlon());
    Serial.print("Altitud:  ");
    Serial.println(gnss.getalt());
  }else{
    Serial.println("No GNSS information read...");
  }
}

