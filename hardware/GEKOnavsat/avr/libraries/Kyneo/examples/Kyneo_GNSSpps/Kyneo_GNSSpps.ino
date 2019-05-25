/**************************************************************************************************
 * Kyneo PPS signal Example.
 * 
 * This sketch will Light up a LED using the PPS signal obtained from the GNSS services
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

int LedTime = 100;  //milliseconds                      //Variable definition

void setup()
{
  Serial.begin(9600);                                   // Boards baudrate
  
  pinMode(GNSS_1PPS, INPUT);                            // Configure PPS signal pin as input
  pinMode(GP_LED_0, OUTPUT);                            // Configure LED0 as output
  
  Serial.println("Setup done\n");
}

void loop()
{
  if(digitalRead(GNSS_1PPS)){                           // Read PPS signal                      
    digitalWrite(GP_LED_0, HIGH);                      
    //Serial.println("LED ON!!");
    delay(LedTime);                              
    digitalWrite(GP_LED_0, LOW);
    //Serial.println("LED Off...");
  }
}

