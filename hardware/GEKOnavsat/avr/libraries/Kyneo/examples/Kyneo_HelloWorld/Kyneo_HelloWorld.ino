/**************************************************************************************************
 * Kyneo Helloworld Example
 * 
 * This sketch prints messages throught terminal and blinks the boards LEDs
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
#include <KyneoUtils.h>


void setup() {
  
  Serial.begin(9600);                                   //Initialize the Serial port 
  
  Serial.println("Hello world");                        //Print message throught the Serial port
  pinMode(13, OUTPUT);                                  //Set pin mode where the LED is
  delay(3000);                                          //Make the program wait
  Serial.println("Let's blink some LEDs");              //Print message throught the Serial port

  pinMode(GP_LED_0, OUTPUT);                            //LEDs digital output 0 set
  pinMode(GP_LED_1, OUTPUT);                            //LEDs digital output 1 set
}

void loop() {
 
  digitalWrite(GP_LED_0, HIGH);                        // set the LED on
  delay(500);
  digitalWrite(GP_LED_1, HIGH);                        // set the LED on
  delay(500);
  digitalWrite(GP_LED_0, LOW);                         // set the LED off
  delay(500);
  digitalWrite(GP_LED_1, LOW);                         // set the LED off
  delay(500);

}
