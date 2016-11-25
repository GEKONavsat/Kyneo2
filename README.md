# GEKONavsat Hardware Specification for Arduino
This repository holds all the files for GEKO 3rd party Arduino-compatible board Kyneo v2 (12MHz, Vcc 3.3V).
The repository is structured following the recommendations in arduino.cc

Instructions:

	- If it's opened, close the Arduino IDE
	- Open the "hardware" folder and copy "GEKOnavsat" into your user's /Documents/Arduino/hardware folder.
	- Open the "libraries" folder and copy all it's content into your user's /Documents/Arduino/libraries folder.
	- Open the Arduino IDE again and... ready to work with Kyneo

Last tested Arduino IDE Version: 1.6.12

### Kyneo

Kyneo uses ATmega1284P. We have adapted the library "mighty-1284p" originally written by maniacbug:
https://github.com/maniacbug/mighty-1284p/

Arduino official web points to the following repo for using mega1284P:
https://github.com/JChristensen/mighty-1284p

Platforms available:
- Kyneo  - 1st version of Kyneo (16MHz, Vcc 5V), which comes with optiboot bootloader preloaded. (not compatible 
with these libraries; look for the Kyneo1 specific libraries)
- Kyneo2 - 2nd version of Kyneo (12MHz, Vcc 3.3V), which comes with optiboot bootloader preloaded.