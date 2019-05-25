/**************************************************************************************************
 * Kyneo IMU speed test example
 * 
 * This sketch will perform a speed test of the sensor reading runtime, it will show the reading
 * time of the IMU, sensor calibration and sensor fusion
 * 
 * Created by GEKO Navsat S.L. for Kyneo V2.0 board
 * 
 * This example is free software, given under a GPL3 license
 * 
 * KYNEO is a product designed by GEKO Navsat S.L. in Europe http://www.gekonavsat.com
 * For further information, visit: http://kyneo.eu/
 * 
 *************************************************************************************************/
#include <KyneoBoard.h>                                    // GekoNavsat libraries
#include <FreeIMU.h>

FreeIMU kyneoIMU;                                          // Object definition

int LoopRate = 5000;  //milliseconds                       // Variable definition
int raw[11]; 
char str[512];
float val[9], q[4];
unsigned long start, stop;


void setup() { 
  
  Serial.begin(9600);                                      // Initial setup
  Serial.println("Kyneo IMU velocity test");
  
  Wire.begin();
  kyneoIMU.init();

  Serial.println("Setup done\n");
}

void loop() {
  
  Serial.println("Testing RAW reading speed:");
  start = micros();
  for(int i=0; i<1000; i++) {                              // Getting values
    kyneoIMU.getRawValues(raw);
  }
  stop = micros();
  Serial.print("--> result: ");
  Serial.print((stop - start) / 1024);
  Serial.print(" microseconds -- ");
  Serial.print(((stop - start) / 1024.0) / 1000.0);        // Write values
  Serial.println(" milliseconds");
  
  
  Serial.println("Testing CALIBRATED reading speed");
  start = micros();
  for(int i=0; i<1024; i++) {
    kyneoIMU.getValues(val);
  }
  stop = micros();
  Serial.print("--> result: ");
  Serial.print((stop - start) / 1024);
  Serial.print(" microseconds --");
  Serial.print(((stop - start) / 1024.0) / 1000.0);
  Serial.println(" milliseconds");
  
  
  Serial.println("Testing SENSOR FUSION speed:");
  start = micros();
  for(int i=0; i<1024; i++) {
    kyneoIMU.getQ(q);
  }
  stop = micros();
  Serial.print("--> result: ");
  Serial.print((stop - start) / 1024);
  Serial.print(" microseconds -- ");
  Serial.print(((stop - start) / 1024.0) / 1000.0);
  Serial.println(" milliseconds");
  
  Serial.println("Test finished, it will be performed again");
  Serial.println("----");
  
  delay(LoopRate);
}
