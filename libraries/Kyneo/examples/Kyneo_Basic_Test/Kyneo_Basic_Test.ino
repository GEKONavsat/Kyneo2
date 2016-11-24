/**************************************************************************************************
 * Kyneo Basic Test Example.
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
 * This simple sketch reads the battery voltage, sends the value through the Serial port and lights 
 * both LEDs in sequence.
 *************************************************************************************************/

#include <KyneoBoard.h>
#include <KyneoUtils.h>

void setup()
{
  boardInit();            // Initialize batt measeurement and LEDs as outputs
  Serial.begin(9600);     // Select your desired baudrate
}

void loop()
{
  Serial.print("Battery level (mV): ");
  Serial.println(battLevel());          // Vbattery in mV. 
  knightrider(300);                     // Funny LEDs testing (choose your prefered delay time)
}

void boardInit()
{
  battInit();                       // Initializes battery measurement
  pinMode(GP_LED_0, OUTPUT);        // Digital output 0
  pinMode(GP_LED_1, OUTPUT);        // Digital output 1
}

void knightrider(int t){
  digitalWrite(GP_LED_0, HIGH);     // LED0 ON.
  delay(t);
  digitalWrite(GP_LED_1, HIGH);     // LED1 ON.
  delay(t);
  digitalWrite(GP_LED_0, LOW);      // LED0 OFF.
  delay(t);
  digitalWrite(GP_LED_1, LOW);      // LED1 OFF.
  delay(t);
}
