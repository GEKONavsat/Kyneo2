/**************************************************************************************************
 * \file 	NMEAparser.h
 *
 * \brief	NMEA frames parsing functions.
 *
 * \author	GEKO NavSat S.L. <info@gekonavsat.com>
 * \date	04/11/2016
 * \version	2
 * \modifications: None (first version)
 *************************************************************************************************/ 

#ifndef __NMEAPARSER_H__
#define __NMEAPARSER_H__

#include <stdio.h>
#include "Arduino.h"
#include "KyneoBoard.h"

#define HDOP_TH       3					//HDOP threshold. Under 3.0 means good precision.
#define MAX_NMEA_SIZE 100				// NMEA frame maximum length

class NMEAparser{
	public:
		NMEAparser();						// Default constructor
	  	uint32_t getutime();				// gets time in Unix format
	  	void gettime(char *frame);			// gets time in char format
	  	void getdate(char *frame);			// gets date in char format
	  	float getlat();						// gets latitude value
	  	float getlon();						// gest longitude value
	  	int16_t getalt();					// gets altitude value
	  	uint16_t gethdg();					// gets heading value
  
	  	uint32_t NMEATimeToUnixTime(char* timeField, char* dateField);		// calculates Unix time format out of date and time data from the NMEA frame
	  	float NMEACoordToDegrees(char* valueField, char hemisphere);		// transforms coordinate information to degrees
	  	int parseNMEA(char* nmea);											// parses NMEA frame info and writes it to object parameters
	  	int parseRMC(char *fields_str);										// parses RMC frame info and writes it to object parameters
	  	int parseGGA(char *fields_str);										// parses GGA frame info and writes it to object parameters
	  	int parseGLL(char *fields_str);										// parses GLL frame info and writes it to object parameters
		
	private:
	  	uint32_t utime;														// Unix time
	  	char _time[7];														// Time in char format
	  	char _date[7];														// Date in char format
	  	float lat;															// Latitude
	  	float lon;															// Longitude
	  	int16_t alt;														// Altitude
	  	uint16_t hdg;														// Heading
	  	void shortString(char *initframe, char *destframe, int lon);		// Auxiliary function, quite similar to strcpy()
};

#endif