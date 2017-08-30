/**************************************************************************************************
 * Kyneo GNSS bypass Example.
 * 
 * This sketch will print the raw measured by the kyneo boards GNSS sensor, as well as accept PMTK 
 * commands for the GNSS sensor modification.
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
#include <KyneoBoard.h>                                                   //GekoNavsat libraries
#include <KyneoGNSS.h>

KyneoGNSS gnss;                                                           //Object definition

char c;                                                                   //Variable definition

void setup()
{
  Serial.begin(9600);                                                     // Boards baudrate
  Serial1.begin(9600);                                                    // GNSS modules baudrate

  //set rate divider(GPGLL, GPRMC, GPVTG, GPGGA, GPGSA, GPGSV, PMTKCHN). 
  gnss.setNMEAOutput(    0,     1,     5,     1,     0,     0,       0);  //GPRMC 1Hz, GPVTG 0.2Hz

  Serial.println("Setup done\n");
}

void loop()
{
  while(Serial1.available()){                                             //Serial1(GNSS) to Serial0 (USB)
    c = Serial1.read();
    Serial.print(c);
  }
  while(Serial.available()){                                              //Serial0 (USB) to Serial1(GNSS)
    c = Serial.read();
    Serial1.print(c);
  }
}

