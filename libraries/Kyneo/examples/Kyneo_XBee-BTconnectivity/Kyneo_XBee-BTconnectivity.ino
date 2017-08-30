/**************************************************************************************************
 * Kyneo XBee-BT connectivity Example
 * 
 * This sketch will send a random number from an xBee to another equaly configured one. It requires
 * two xBee modules, one conected to the kyneo and the other to the computer via an FTDI adapter. 
 * 
 * Instructions: Charge this program to the kyneo and unplug it. Connect one xBee to the kineo and
 * the other to the FTDI adapter. Plug the FTDI on the computer, open arduino, select the com port
 * and start the serial monitor. Start the kyneo, you should see the random numbers generated on 
 * the serial monitor.
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

int LoopRate = 500;  //milliseconds                     //Variable definition
float RandomNum;
int RandomMax = 100;

void setup() {
  Serial.begin(9600);                                   //Initialize the Serial port 
}  

void loop() {
  
  RandomNum = random(RandomMax);                        //Random number generator
  Serial.println(RandomNum);

  delay(LoopRate);
}  
