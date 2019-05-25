/**************************************************************************************************
 * Kyneo Quaternion example
 * 
 * This sketch prints the quaternions used to show for orientation and rotations of the system.
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

FreeIMU kyneoIMU;                                       //Object definition

int LoopRate = 10;  //milliseconds                      //Variable definition
float q[4];
int counter = 9;


void setup() {
  
  Serial.begin(9600);
  
  Wire.begin();
  kyneoIMU.init();
  
  Serial.println("Setup done\n");
}

void loop() 
{ 
  counter++;
  if(counter == 10){                                                        // Prints column titles every 10 lines
    counter = 0;  
    Serial.println("q0\tq1\tq2\tq3"); 
  }

  kyneoIMU.getQ(q);
  Serial.print(q[0]);
  Serial.print("\t");
  Serial.print(q[1]);
  Serial.print("\t");
  Serial.print(q[2]);
  Serial.print("\t");
  Serial.println(q[3]);
  
  delay(LoopRate);
}
