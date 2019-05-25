/**************************************************************************************************
 * Kyneo GNSS position Example.
 * 
 * This sketch will print the position measured by the kyneo board based in the GNSS sensor.
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
#include <KyneoGNSS.h>

KyneoGNSS gnss;                                         //Object definition

float lat, lon;                                         //Variable definition                                 
int alt;
char hour[10], day[10];

void setup()
{
  Serial.begin(9600);                                                              // Boards baudrate
  Serial1.begin(9600);                                                             // GNSS modules baudrate

  //set rate divider(GPGLL, GPRMC, GPVTG, GPGGA, GPGSA, GPGSV, PMTKCHN). 
  gnss.setNMEAOutput(    0,     1,     0,     1,     0,     0,       0);           //GPRMC 1Hz, GPGGA 1Hz
  
  Serial.println("Setup done\n");
}

void loop()
{
  if(gnss.getLatLon(lat, lon) == 2){                                               // Data adquisition
    gnss.getdate(day);
    gnss.gettime(hour);
    alt = gnss.getalt();
    
    Serial.println("--------------------");                                        //Data print
    Serial.print("Fecha:    ");
    Serial.println(day);
    Serial.print("Hora:     ");
    Serial.println(hour);
    Serial.print("Latitud:  ");
    Serial.println(lat,6);
    Serial.print("Longitud: ");
    Serial.println(lon,6);
    Serial.print("Altitud:  ");
    Serial.println(alt);
  }else{
    Serial.println("No GNSS information read...");
  }
}

