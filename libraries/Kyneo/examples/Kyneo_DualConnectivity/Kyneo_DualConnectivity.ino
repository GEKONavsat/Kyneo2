/**************************************************************************************************
 * Kyneo Dual Connectivity Example
 * 
 * This sketch will send a random number from a bluetooth beacon conected to the kyneo. 
 * 
 * Instructions: In order to visualize the data sent, another bluetooth enabled device will be needed,
 * such a smartphone. Any bluetooth terminal should provide the tools to read the data.
 * 
 * 
 * PrimeSieve
 * Paul Badger 2009
 * Generates prime numbers less than 63,001 as shown
 * To extend just add more primes to prime table (and choose a larger data type)
 * The program will generate primes up to the square of the last prime table entry
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
#include <KyneoBoard.h>                                 // GekoNavsat libraries
#include <SoftwareSerial.h>                             // Other libraries

SoftwareSerial BTSerial(0, 1); // Rx, Tx                // Object definition

int LoopRate = 500;                                     // Variable definition
byte primes[]={ 
    2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101,
    103, 107, 109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193, 197, 
    199, 211, 223, 227, 229, 233, 239, 241, 251};
const unsigned int TopPrimeIndex = sizeof(primes) - 1;      
const unsigned long TopPrimeSquared = (long)primes[TopPrimeIndex] * (long)primes[TopPrimeIndex];
int primeFlag;


void setup() {
  Serial.begin(9600);                                   // Initialize the Serial port 
  BTSerial.begin(9600);
  BTSerial.println("Setup done /n");
}  

void loop() {

  for (long x = 1; x < TopPrimeSquared; x++){
    for (long j=0; j < TopPrimeIndex; j++){
      
      primeFlag = true;
      if (x == primes[j]) break;
      if (x % primes[j] == 0){                          // If the test number from prime table
        primeFlag = false;                              // Then test number is not prime
        break;
      }
    }
    
    if (primeFlag == true){                             // Found a prime - print it
      BTSerial.println(x);
      delay(LoopRate);
    }else{
      Serial.println(x);
    }
  }
}  
