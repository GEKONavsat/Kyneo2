/**************************************************************************************************
 * Kyneo test bench Example
 * 
 * This serves as a test bench in order to check the readings of the boards sensors, the imput
 * commands are the following:
 * 
 * A.- Euler Angles values
 * Q.- Quaternion values
 * R.- Raw IMU sensors data
 * B.- Battery Voltage
 * L.- Blinking LEDs
 * G.- Location and time information
 * N.- Last received NMEA frame
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
#include <KyneoBoard.h>                                                           // GekoNavsat libraries
#include <KyneoGNSS.h>
#include <KyneoUtils.h>
#include <FreeIMU.h>

FreeIMU kyneoIMU;                                                                 // Object definition
KyneoGNSS gnss;         
KyneoUtils util;  

int LoopRate = 10;  //milliseconds                                                // Variable definition
int alt;
int gnss_idx = 0, newFrame = 0, options = 0, count = 0;
int headprint = 10, counter = 9, ledFactor = 10;                                  // If you want to read the hidden message, set ledFactor to 1
int raw[9];
float lat, lon;
float att[3],q[4];
char cmd, a;
char frame[100], day[10], hour[10];

void setup()
{
  Serial.begin(9600);
  Serial.print("Setting up... ");

  util.battInit();
  Wire.begin();
  kyneoIMU.init();
  pinMode(GP_LED_0, OUTPUT);
  pinMode(GP_LED_1, OUTPUT);

  Serial1.begin(9600);  // Default baudrate to connect with GNSS module
  delay(100);
  //set rate divider(GPGLL, GPRMC, GPVTG, GPGGA, GPGSA, GPGSV, PMTKCHN). 
  gnss.setNMEAOutput(    0,     1,     0,     1,     0,     0,       0);         // Period of each NMEA message
                                                                          
  Serial.println("done!");
  Serial.println("--------------------------------------------");
  Serial.println("Welcome to the Kyneo testbench by GekoNavsat!");
}

void loop() {
  if(options == 0){
    Serial.println();
    Serial.println("Choose one of the following options:");
    Serial.println("    ");
    Serial.println("    H.- Hello");
    Serial.println("    A.- Euler Angles values");
    Serial.println("    Q.- Quaternion values");
    Serial.println("    R.- Raw IMU sensors data");
    Serial.println("    B.- Battery Voltage");
    Serial.println("    L.- Blinking LEDs");
    Serial.println("    G.- Location and time information");
    Serial.println("    N.- Last received NMEA frame");
    Serial.println("    - Send any command to stop");
    Serial.println();
    Serial.println("*commands must be sent with -NO LINE ENDING-");
    Serial.println();
    
    while(!Serial.available());                                                   // Waits for user command
    while(Serial.available()){
      if(count++ == 0)cmd = Serial.read();
    }
    count = 0;
    options = 1;
  }else{

    if(cmd == 'h'|| cmd == 'H') {
      Serial.println("Hello user!");
      
    }else if (cmd == 'a' || cmd == 'A'){
      while(!Serial.available()){
        kyneoIMU.getYawPitchRoll(att);
        Serial.print("Yaw: ");
        Serial.print(att[0]);
        Serial.print(" Pitch: ");
        Serial.print(att[1]);
        Serial.print(" Roll: ");
        Serial.print(att[2]);
        Serial.println("");
      }
      
    }else if (cmd == 'q' || cmd == 'Q'){

      while(!Serial.available()){
        kyneoIMU.getQ(q);
        Serial.print("  q0: ");
        Serial.print(q[0]);
        Serial.print("  q1: ");
        Serial.print(q[1]);
        Serial.print("  q2: ");
        Serial.print(q[2]);
        Serial.print("  q3: ");
        Serial.println(q[3]);
      }
      
    }else if (cmd == 'r' || cmd == 'R'){
      while(!Serial.available()){
        counter++;
        if(counter == 10){                                                      // Prints column titles every 10 lines
          counter = 0;
          Serial.println("acc_x\tacc_y\tacc_z\tgyr_x\tgyr_y\tgyr_z\tmag_x\tmag_y\tmag_z\tTemp\tAlt");  
        }
  
        kyneoIMU.getRawValues(raw);
        Serial.print((float)raw[0]/2048, 2);    // acc_x
        Serial.print("\t");
        Serial.print((float)raw[1]/2048, 2);    // acc_y
        Serial.print("\t");
        Serial.print((float)raw[2]/2048, 2);    // acc_z
        Serial.print("\t");
        Serial.print((float)raw[3]/16.384, 2);  //gyr_x
        Serial.print("\t");
        Serial.print((float)raw[4]/16.384, 2);  //gyr_y
        Serial.print("\t");
        Serial.print((float)raw[5]/16.384, 2);  //gyr_z
        Serial.print("\t");
        Serial.print((float)raw[6]/1.707, 1);   //mag_x
        Serial.print("\t");
        Serial.print((float)raw[7]/1.707, 1);   //mag_y
        Serial.print("\t");
        Serial.print((float)raw[8]/1.707, 1);   //mag_z
        Serial.print("\t");
        Serial.print(kyneoIMU.baro.getTemperature(MS561101BA_OSR_4096), 1);    // Temperature reading
        Serial.print("\t");
        Serial.println(kyneoIMU.getBaroAlt(), 2);   //pres
      }
      
    }else if (cmd == 'b' || cmd == 'B'){
      while(!Serial.available()){
        Serial.print("Battery level (mV): ");
        Serial.println(util.battLevel());
        delay(500);
      }
      
    }else if (cmd == 'g' || cmd == 'G'){
      while(!Serial.available()){
        gnss_idx = gnss.getLatLon(lat, lon);
        if(gnss_idx == 2){
          gnss.getdate(day);
          gnss.gettime(hour);
          alt = gnss.getalt();
      
          Serial.println("--------------------");
          Serial.print("Fecha:    ");
          Serial.println(day);
          Serial.print("Hora:     ");
          Serial.println(hour);
          Serial.print("Latitud:  ");
          Serial.println(lat,6);
          Serial.print("Longitud: ");
          Serial.println(lon,6);
          Serial.print("Altitud:  ");
          Serial.println(alt);
        }else{
          Serial.println("No GNSS information read...");
        }
      }
      
    }else if (cmd == 'n' || cmd == 'N'){
      while(!Serial.available()){
        if(gnss.getSingleNMEA(frame, 100)!=0){
          Serial.print("NMEA received: ");
          Serial.print(frame);
        }else{
          Serial.println("No NMEA frame received");
        }
      }

    }else if (cmd == 'l' || cmd == 'L'){
      while(!Serial.available()){                                          // Blinks some cool morse code ;-)
        Serial.println("Blinking LEDs!, (wair for whole message)");
        digitalWrite(GP_LED_0, HIGH);   // LED0 ON.
        delay(750/ledFactor);
        digitalWrite(GP_LED_0, LOW);    // LED0 OFF.
        delay(250/ledFactor);
        digitalWrite(GP_LED_0, HIGH);   // LED0 ON.
        delay(750/ledFactor);
        digitalWrite(GP_LED_0, LOW);    // LED0 OFF.
        delay(250/ledFactor);
        digitalWrite(GP_LED_0, HIGH);   // LED0 ON.
        delay(250/ledFactor);
        digitalWrite(GP_LED_0, LOW);    // LED0 OFF.
        delay(750/ledFactor);

        digitalWrite(GP_LED_0, HIGH);   // LED0 ON.
        delay(250/ledFactor);
        digitalWrite(GP_LED_0, LOW);    // LED0 OFF.
        delay(750/ledFactor);

        digitalWrite(GP_LED_0, HIGH);   // LED0 ON.
        delay(750/ledFactor);
        digitalWrite(GP_LED_0, LOW);    // LED0 OFF.
        delay(250/ledFactor);
        digitalWrite(GP_LED_0, HIGH);   // LED0 ON.
        delay(250/ledFactor);
        digitalWrite(GP_LED_0, LOW);    // LED0 OFF.
        delay(250/ledFactor);
        digitalWrite(GP_LED_0, HIGH);   // LED0 ON.
        delay(750/ledFactor);
        digitalWrite(GP_LED_0, LOW);    // LED0 OFF.
        delay(750/ledFactor);

        digitalWrite(GP_LED_0, HIGH);   // LED0 ON.
        delay(750/ledFactor);
        digitalWrite(GP_LED_0, LOW);    // LED0 OFF.
        delay(250/ledFactor);
        digitalWrite(GP_LED_0, HIGH);   // LED0 ON.
        delay(750/ledFactor);
        digitalWrite(GP_LED_0, LOW);    // LED0 OFF.
        delay(250/ledFactor);
        digitalWrite(GP_LED_0, HIGH);   // LED0 ON.
        delay(750/ledFactor);
        digitalWrite(GP_LED_0, LOW);    // LED0 OFF.
        delay(1500/ledFactor);

        
        digitalWrite(GP_LED_1, HIGH);   // LED0 ON.
        delay(750/ledFactor);
        digitalWrite(GP_LED_1, LOW);    // LED0 OFF.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, HIGH);   // LED0 ON.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, LOW);    // LED0 OFF.
        delay(750/ledFactor);

        digitalWrite(GP_LED_1, HIGH);   // LED0 ON.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, LOW);    // LED0 OFF.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, HIGH);   // LED0 ON.
        delay(750/ledFactor);
        digitalWrite(GP_LED_1, LOW);    // LED0 OFF.
        delay(750/ledFactor);

        digitalWrite(GP_LED_1, HIGH);   // LED0 ON.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, LOW);    // LED0 OFF.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, HIGH);   // LED0 ON.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, LOW);    // LED0 OFF.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, HIGH);   // LED0 ON.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, LOW);    // LED0 OFF.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, HIGH);   // LED0 ON.
        delay(750/ledFactor);
        digitalWrite(GP_LED_1, LOW);    // LED0 OFF.
        delay(750/ledFactor);

        digitalWrite(GP_LED_1, HIGH);   // LED0 ON.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, LOW);    // LED0 OFF.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, HIGH);   // LED0 ON.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, LOW);    // LED0 OFF.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, HIGH);   // LED0 ON.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, LOW);    // LED0 OFF.
        delay(750/ledFactor);

        digitalWrite(GP_LED_1, HIGH);   // LED0 ON.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, LOW);    // LED0 OFF.
        delay(250/ledFactor);
        digitalWrite(GP_LED_1, HIGH);   // LED0 ON.
        delay(750/ledFactor);
        digitalWrite(GP_LED_1, LOW);    // LED0 OFF.
        delay(750/ledFactor);

        digitalWrite(GP_LED_1, HIGH);   // LED0 ON.
        delay(750/ledFactor);
        digitalWrite(GP_LED_1, LOW);    // LED0 OFF.
        delay(2000/ledFactor);
      }
      
    }else{
      Serial.println("Command not found");
    }
    while(Serial.available()) a = Serial.read();                               // Serial buffer flushing
    options = 0;
  }
}
