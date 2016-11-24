/**************************************************************************************************************************
 * \file 	KyneoGNSS.h
 *
 * \brief	Advanced library for GMS-G9 module, based on MT3333 core, the GNSS device present in Kyneo.
 *
 * \author	GEKO NavSat S.L. <info@gekonavsat.com>
 * \date	12/01/2015
 * \version	2.0         (-- especific for Kyneo2; NOT VALID FOR KYNEO 1.0 -- )
 * \modifications: Includes awaiting loops for response from the GNSS module.
 * \               Main serial comm with GNSS is via Serial1.
 * \               Second serial comm is via SoftwareSerial port (called DportSerial, mainly for RTCM corrections feeding).
 *
 ***************************************************************************************************************************/

#ifndef KYNEOGNSS_H_
#define KYNEOGNSS_H_

#include <stdio.h>
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "HardwareSerial.h"
#include "KyneoBoard.h"
#include "NMEAparser.h"

#define KYNEOGNSS_DEBUG		1		// 0 disabled, 1 errors, 2 errors and info.
#define KYNEOGNSS_VERSION   2		// KyneoGNSS Library version

//DEFAULT MODULE CONFIGURATION
#define DFLT_GNSS_BAUDRATE	9600u	// Default is 9600 bauds, 8 bit data, 1 stop bit, no parity.
#define DFLT_FIX_RATE_MS	1000u	// Fix frequency (update rate) is 1 Hz (1/1000 ms)
#define DFLT_GPGLL_DIV		0u		// Default values for NMEA sentences has been chossen according to the datasheet information
#define DFLT_GPRMC_DIV		1u
#define DFLT_GPVTG_DIV		1u
#define DFLT_GPGGA_DIV		1u
#define DFLT_GPGSA_DIV		1u
#define DFLT_GPGSV_DIV		5u
#define DFLT_PMTKCHN_DIV	0u
#define TIMEOUT				5000	// Maximum time (millis) waiting for response from GNSS module

//NMEA SENTENCES MAXIMUM LENGTH
#define GPGLL_MAX_LENGTH	40u
#define GPRMC_MAX_LENGTH	68u
#define GPVTG_MAX_LENGTH	41u
#define GPGGA_MAX_LENGTH	72u		// Review
#define GPGSA_MAX_LENGTH	68u
#define GPGSV_MAX_LENGTH	68u

// Gms-g9 MTK PROTOCOL COMMANDS
#define MAX_CMD_SIZE		64u
#define MAX_NMEA_DIV		5u
#define MIN_FIX_RATE_MS		100u
#define MAX_FIX_RATE_MS		10000u

// Gms-g9 DEFAULT CORRECT ANSWERS
#define ANS220			   "$PMTK001,220,3*30"			// Answer to setFixRate() (for any valid rate value)
#define ANS250			   "$PMTK001,250,3*37"			// Answer to setDPortBaudrate() (for any parameters introduced)
#define ANS301			   "$PMTK001,301,3*32"			// Answer to SetDGPSMode() (for any mode selected)
#define ANS313             "$PMTK001,313,3*31"			// Answer to SBAS Enable or Disable
#define ANS314             "$PMTK001,314,3*36"			// Answer to setNMEAOutput()
#define ANS353_01		   "$PMTK001,353,3,0,1,0,2*36"	// Answer to setSearchMode(0, 1)
#define ANS353_10		   "$PMTK001,353,3,1,0,0,1*35"	// Answer to setSearchMode(1, 0)
#define ANS353_11		   "$PMTK001,353,3,1,1,0,3*36"	// Answer to setSearchMode(1, 1)
#define AND501fr		   "$PMTK501,"					// First part of the answer to getDGPSMode()
#define	ANS513			   "$PMTK513,1*28"				// Answer to checkSBASEnable()

// Gms-g9 RESTART MODES
#define HOT_START		1
#define WARM_START		2
#define COLD_START		3
#define FULL_COLD_START	4

// Gms-g9 DGPS Correction Modes
#define NoDGPS			0
#define RTCM			1
#define SBAS			2

// Gms-g9 Aux Port DGPS Frame Types
#define DportNONE		0
#define DportRTCM		1
#define DportNMEA		2

class KyneoGNSS{
	public:
		KyneoGNSS();

		// Basic functions
		int version();										// library version
		void setDfltConfig();								// sets GNSS module with default configuration
		void restart(uint8_t start_mode);					// sets the restart command
		void enable();										// enables GNSS module operation
		void disable();										// disables GNSS module operation
													
		// Main serial port functions
		void setBaudrate(uint32_t bauds);					// sets main port baudrate
		uint32_t getBaudrate();								// gets main port baudrate value
		void sendCommand(char *cmd);						// send an specific command to the GNSS module
		void sendChar(char c);								// sends a single char to the GNSS module
		
		// Dport (SoftwareSerial port) functions
		void DportInit(uint32_t bauds = 9600);										// inits Dport SW Serial port
		int8_t setDPortBaudrate(uint8_t modeIn, uint8_t modeOut, uint32_t bauds);	// sets Dport SW Serial port baudrate
		uint32_t getDportBaudrate();												// gets Dport SW seiral port baudrate
	    void DportFlush();															// flushes Dport buffer	
		char DportAvailable();														// checks Dport receiving buffer
		char DportRead();															// reads Dport receiving buffer
		char DportWrite(uint8_t byte);												// sends a char via Dport
		char DportPrint(const char* buf);											// sends a char* via Dport
		char DportPrintln(const char* buf);											// sends a char* (with an added newline feed) via Dport
			
		// GNSS especific functions
		int8_t setFixRate(uint16_t fixRate);																								// sets Fix Rate (how much time runs between each position calculated)
		uint16_t getFixRate();																												// gets Fix Rate (how much time runs between each position calculated)
		int8_t setNMEAOutput(uint8_t GPGLL, uint8_t GPRMC, uint8_t GPVTG, uint8_t GPGGA, uint8_t GPGSA, uint8_t GPGSV, uint8_t channel);	// sets NMEA output rate dividers (how many fixes pass between NMEA sendings)
		int8_t DisNMEAOutput();																												// disables NMEA sendings
		
		int8_t setSearchMode(uint8_t GPS_EN, uint8_t GLONASS_EN);					// sets constellation to be used (GPS, GLONASS or both)
		
		int8_t SetDGPSMode(uint8_t mode);											// sets dinamic GNSS technique to be used (0=none, 1=RTCM, 2=SBAS)
		int8_t getDGPSMode();														// gets DGPS mode selected
		
		int8_t setRTCMConfig(uint32_t bauds);										// sets a complete RTCM configuration
		
		int8_t EnSBAS();															// enables SBAS (not activated, just enabled)
		int8_t DisSBAS();															// disable SBAS (needs to be ON before activated)
		int8_t checkSBASEnable();													// check if SBAS is enabled
		int8_t setSBASConfig();														// sets a complete SBAS configuration
		
		// NMEA reading functions
		int getSingleNMEA(char *frame, int length);									// reads a single NMEA frame from GNSS module (default TIMEOUT)
		int getSingleNMEA(char *frame, int length, unsigned int timeout);			// reads a single NMEA frame from GNSS module (specific timeout)
		
		int getNMEAtype(char *frame, int length, char *type);						// reads next NMEA frame of specified type
		int getNMEAlatlon(char *frame, int length);									// reads next NMEA and returns it if it's RMC, GGA or GLL (those with position info)
		int getGSV(char *frame, int length);										// reads next GSV frame
		int getGSA(char *frame, int length);										// reads next GSA frame
		int getVTG(char *frame, int length);										// reads next VTG frame
		int getGLL(char *frame, int length);										// reads next GLL frame
		int getRMC(char *frame, int length);										// reads next RMC frame
		int getGGA(char *frame, int length);										// reads next GGA frame
		int getNMEA(char *frame, int length);										// reads next NMEA frame (whatever it is)
		int getNMEA(char *frame, int length, unsigned int timeout, int numFrames);
				
				// using NMEA parser functions
		int getLatLon(float &lat, float &lon);										// extracts latitude and longitude from NMEA frame
		uint32_t getutime();														// gets time in Unix format
	  	void gettime(char *frame);													// gets time in char format
	  	void getdate(char *frame);													// gets date in char format
	  	float getlat();																// gets latitude value
	  	float getlon();																// gets longitude value
	  	int16_t getalt();															// gets altitude value
	  	uint16_t gethdg();															// gets heading value
		
		// Auxiliary function
		char getChecksum(char *s, uint8_t length);									// calculates checksum value of an NMEA sentence
		
	private:
		SoftwareSerial	DportSerial;			// Software serial port to communicate with GNSS Module Dport (RTCM) (before it was GNSSerial)
		NMEAparser 		nmea;					// nmea object to use the NMEAparser library
		
		uint32_t 		DportBaudrate			// Software serial port baudrate (RTCM)
		uint32_t 		baudrate;				// Main port baudrate
		
		uint16_t 		fixRate_ms;				// Fix Interval in milliseconds. From 100 (10 Hz) to 10000 (0.1 Hz) or 0 (disabled).
		uint8_t 		GPGLLdiv;				// GPGLL rate divider 
		uint8_t 		GPRMCdiv;				// GPRMC rate divider
		uint8_t 		GPVTGdiv;				// GPVTG rate divider
		uint8_t 		GPGGAdiv;				// GPGGA rate divider
		uint8_t 		GPGSAdiv;				// GPGSA rate divider
		uint8_t 		GPGSVdiv;				// GPGSV rate divider
		uint8_t 		PMTKCHNdiv;				// Channel status rate divider.
		
		int compareStrings(char *a, char *b, int lon);								// Similar to strcmp, just little bit enhanced
		int8_t checkResponse(char *x);												// Checks response matches with the expected one
		int AuxGetSingleNMEA(char *frame, int length, unsigned int timeout);		// core of getSingleNMEA() family functions
		int AuxGetNMEA(char *frame, int length, unsigned int timeout, int numFrames);
};

#endif /* KYNEOGNSS_H_ */