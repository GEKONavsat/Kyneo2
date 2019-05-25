/**************************************************************************************************
 * \file KyneoBoard.h
 *
 * \brief	Hardware definitions for Kyneo_v2 board.
 *
 * \author	GEKO NavSat S.L. <info@gekonavsat.com>
 * \date	28/10/2016
 * \version	2.0
 *************************************************************************************************/ 

#ifndef KYNEOBOARD_H_
#define KYNEOBOARD_H_

#define KYNEO_VERSION 2				// Kyneo HW version

// ####################################################################################
// ################################ Pin definitions ###################################
// ####################################################################################

//      Signal		   Arduino Pin	  ATMEGA1284P Pin	Kind 	Description

#define SD_CS_PIN			4		// 		44			OUT		Kyneo SD chip select pin

#define GNSS_RX2			12		// 		13			COM		RX pin to connect with GNSS secondary serial TX pin	(NMEA frames transmission from GNSS)
#define GNSS_TX2			15		// 		16			COM		TX pin to connect with GNSS secondary serial RX pin (diff corrections injection into GNSS)
#define GNSS_1PPS			13		// 		14			IN		GNSS Module Pulse per second signal
#define GNSS_EN_PIN			18		// 		21			OUT		GNSS Enable, by default active high with pull-up to 3,3V
#define GNSS_3DFIX			22		//		25			IN		GNSS indicator of 3D Fix achieved

#define XBEE_SLEEP			14		// 		15			OUT		XBee sleep request
#define XBEE_CTS			21		//		24			COM		XBee Clear to send
#define XBEE_RSSI			23		//		26			IN		XBee RSSI PWM signal

#define BATT_LEVEL_PIN		A7		// 		30			IN		Batt level measurement pin
#define BATT_ADC_CH			7		// 		30			IN		Batt level measurement channel (to use with analogRead() function)

#define GP_LED_0			20		// 		23			I/O		General purpose LED0
#define GP_LED_1			19		// 		22			I/O		General purpose LED1


// ####################################################################################
// ############################# Parameter definitions ################################
// ####################################################################################

#define VCC					3300		// Supply voltage (mV)

#define BATT_RTOP			470000.00	// R value (ohms) of the Resistor placed between Vcc and Batt level pin
#define BATT_RBOTTOM		680000.00	// R value (ohms) of the Resistor placed between Batt level pin and GND
	
#endif /* KYNEOBOARD_H_ */