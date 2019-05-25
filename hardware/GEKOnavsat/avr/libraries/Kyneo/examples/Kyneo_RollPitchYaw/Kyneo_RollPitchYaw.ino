/**************************************************************************************************
 * Kyneo Euler angles Example
 * 
 * This sketch shows the euler angles of the board in the serial monitor or plotter

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
float att[3];
 

void setup() {
  
  Serial.begin(9600);
  
  Wire.begin();
  kyneoIMU.init();
  
  Serial.print("Setup done\n");
}

void loop() {
  kyneoIMU.getYawPitchRoll(att);                          // Sensor read

  Serial.print("Yaw: ");                                // Data Print
  Serial.print(att[0]);
  Serial.print(",");
  Serial.print(" Pitch: ");
  Serial.print(att[1]);
  Serial.print(",");
  Serial.print(" Roll: ");
  Serial.println(att[2]);
  
  delay(LoopRate);
}
