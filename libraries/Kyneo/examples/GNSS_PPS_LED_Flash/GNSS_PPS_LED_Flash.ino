/**************************************************************************************************
 * Kyneo PPS Signal Testing Example.
 * 
 * Created on 22 Nov 2016 by GEKO Navsat S.L.
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
 * This sketch turns ON a LED everytime a pulse arrives on PPS signal (Pulse per Second, sent from GNSS).
 * 
 ******************************************************************************************************/
#include <KyneoBoard.h>

void setup()
{
  Serial.begin(9600);               // Choose your preferred baudrate here.
  pinMode(GNSS_1PPS, OUTPUT);       // Configure PPS signal pin as input
  pinMode(GP_LED_0, OUTPUT);        // Configure LED0 as output
}

void loop()
{
  if(digitalRead(GNSS_1PPS)){       // Everytime a pulse arrives, the LED stays on for some time
    Serial.println("LED ON!!");
    digitalWrite(GP_LED_0, HIGH);
    delay(300);                     // Choose your prefered delay (lower than 1000 or you may loose pulses)
    digitalWrite(GP_LED_0, LOW);
    Serial.println("LED Off...");
  }
}

