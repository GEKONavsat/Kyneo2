#  GEKO Navsat Hardware Specification for Arduino
This repository holds all the files for GEKO 3rd party Arduino-compatible boards.
The repository is structured following the recommendations in arduino.cc

Instructions: Copy the folder GEKOnavsat into your user's /Documents/Arduino/hardware folder.

Last tested Arduino IDE Version: 1.6.12

## GEKO Boards

### Kyneo

Kyneo uses ATmega1284P. We have adapted the library "mighty-1284p" originally written by maniacbug:
https://github.com/maniacbug/mighty-1284p/

Arduino official web points to the following repo for using mega1284P:
https://github.com/JChristensen/mighty-1284p

Platforms available:
- Kyneo  - 1st version of Kyneo (16MHz, Vcc 5V), which comes with optiboot bootloader preloaded.
- Kyneo2 - 2nd version of Kyneo (12MHz, Vcc 3.3V), which comes with optiboot bootloader preloaded.  