/**************************************************************************************************
 * Kyneo Voltage and Temperature Example
 * 
 * This sketch will show the board temperature as well as the battery voltage through the Serial 
 * monitor
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
#include <KyneoUtils.h>

KyneoUtils util;                                        //Object definition
FreeIMU kyneoIMU;

int LoopRate = 3000;  //milliseconds                 //Variable definition


void setup()
{
  Serial.begin(9600);                                                        //Initialization
  
  util.battInit();
  Wire.begin();
  kyneoIMU.init();

  delay(100);
  Serial.println("Setup done");
  delay(100);
  Serial.println("Remember to connect the battery \n");

  util.battLevel(); //first unreal read
}


void loop()
{
  Serial.print("Battery level (mV): ");
  Serial.println(util.battLevel());                                          //Data adquisition and display
  Serial.print("Board Temperature (Cº): ");
  Serial.print(kyneoIMU.baro.getTemperature(MS561101BA_OSR_4096), 1); 
  Serial.println("\n");
  
  delay(LoopRate);
}
