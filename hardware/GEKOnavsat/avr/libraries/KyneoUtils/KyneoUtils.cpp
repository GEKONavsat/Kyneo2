/**************************************************************************************************
 * \file 			KyneoUtils.h
 *
 * \brief			Auxiliary miscellaneous functions for Kyneo2.
 *
 * \author			GEKO NavSat S.L. <info@gekonavsat.com>
 * \date			03/11/2016
 * \version			2
 * \modifications: 	None (despite of the version number, this is the first version)
 *************************************************************************************************/ 

#include "KyneoUtils.h"

/**
 * \fn 		<KyneoUtils()>
 * \pre 	{None.}
 * \brief 	{Default Constructor.}
  * \param 	{void}
 * \return 	{void}
 * \note 	{void}
 */
KyneoUtils::KyneoUtils(){
}

/**
 * \fn 		<rssi()>
 * \pre 	{None.}
 * \brief 				{Reads Xbee RSSI value (usec in HIGH state, from a maximum of 64us).}
 * \param 	{None.}
 * \return 	{void}
 * \note 	{}
 */
unsigned long KyneoUtils::rssi()
{
	unsigned long rssi = auxRssi(XBEE_RSSI, 200);
	
	if(rssi > 64) rssi = 64;
	
	return rssi;
}

/**
 * \fn 		<rssi(int k)>
 * \pre 	{None.}
 * \brief 					{Reads Xbee RSSI value (usec in HIGH state, from a maximum of 64us).}
 * \param 	<int k> 		{Reference maximum value of the signal (tipically k = 100 or k = 1000).}
 * \return 	<unsigned long>	{RSSI value refered to the reference maximum value.}
 * \note 	{The higher the 'k' value, the higher the precision in the returned value.}
 */
unsigned long KyneoUtils::rssi(int k)
{
	unsigned long rssi = auxRssi(XBEE_RSSI, 200);
	
	if(rssi > 64) rssi = 64;
	
	return rssi * k / 64;
}

/**
 * \fn 		<battInit()>
 * \pre 	{None.}
 * \brief 	{Prepares the Kyneo2 board for the battery measurement.}
 * \param 	<void>
 * \return 	<void>
 * \note 	{}
 */
void KyneoUtils::battInit()
{
	pinMode(BATT_LEVEL_PIN, INPUT);  // Prepares Batt Level pin as analog input
  	analogReference(INTERNAL2V56);   // Sets Analog Reference: internally generated 2.56 Volts
}

/**
 * \fn 		<battLevel()>
 * \pre 					{Requires Battery measurement pin initialization (with battInit()).}
 * \brief 					{Prepares the Kyneo2 board for the battery measurement.}
 * \param 	<void>
 * \return 	<float>			{Battery level (mV)}
 * \note 					{Uses INTERNAL2V56 (2.56V) as default voltage reference}
 */
float KyneoUtils::battLevel(){
	return ((float)analogRead(BATT_LEVEL_PIN) * 2560 / 1023) * ( ( BATT_RBOTTOM + BATT_RTOP ) / BATT_RBOTTOM );
}

/**
 * \fn 		<version()>
 * \pre 					{}
 * \brief 					{Returns the library version number.}
 * \param 	<void>
 * \return 	<float>			{Version number}
 * \note 					{}
 */
int version(){
	return KYNEOUTILS_VERSION;
}

/**
 * \fn 		<analogReadwithRef()>
 * \pre 					{Requires analog measurement pin initialization.}
 * \brief 					{Reads an specific analog channel.}
 * \param 					{Analog reference (DEFAULT(3V3), INTERNAL1V1, INTERNAL2V56), analog channel (0 or 1)}
 * \return 	<float>			{Voltage measurement (mV)}
 * \note 					{Uses 3V3 as default reference and channel 1 as default one}
 */
float KyneoUtils::analogReadwithRef(int analogRef, int analogChannel){
  float mV = 0;
  int ref = 0;
  
  if(analogRef == 1)      ref = 3300;
  else if(analogRef == 2) ref = 1100;
  else if(analogRef == 3) ref = 2560;
  else                    ref = 3300;

  if(analogChannel == 0)  mV = ( (float)analogRead(A0) * ref / 1023 );
  else                    mV = ( (float)analogRead(A1) * ref / 1023 );

  return mV;
}

/*-----------------------------------------------------------------------------------------------------------
											PRIVATE FUNTIONS
------------------------------------------------------------------------------------------------------------*/

/**
 * \fn 		<auxRssi(uint8_t pin, unsigned long timeout)>
 * \pre 	{None.}
 * \brief 	{Reads PWM signal pulse length on specific pin (RSSI).}
 * \param 	<pin> 	{Pin where the signal is measured}
 *        	<timeout>	{Maximum time that the function is working. Better if it's about four times the PWM period.}
 * \return 	<unsigned long> {Amount of time (us) that the signal is in HIGH state.}
 * \note 	{}
 */

unsigned long KyneoUtils::auxRssi(uint8_t pin, unsigned long timeout)
{
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t stateMask = (HIGH ? bit : 0);
  unsigned long width = 0; // keep initialization out of time critical area
  
  // convert the timeout from microseconds to a number of times through
  // the initial loop; it takes 16 clock cycles per iteration.
  unsigned long numloops = 0;
  unsigned long maxloops = microsecondsToClockCycles(timeout) / 16;
  
  // wait for any previous pulse to end
  while ((*portInputRegister(port) & bit) == stateMask)
    if (numloops++ == maxloops)
      return timeout;
  
  // wait for the pulse to start
  while ((*portInputRegister(port) & bit) != stateMask)
    if (numloops++ == maxloops)
      return 0;
  
  // wait for the pulse to stop
  while ((*portInputRegister(port) & bit) == stateMask) {
    if (numloops++ == maxloops)
      return timeout;
    width++;
  }

  // convert the reading to microseconds. The loop has been determined
  // to be 20 clock cycles long and have about 16 clocks between the edge
  // and the start of the loop. There will be some error introduced by
  // the interrupt handlers.
  return clockCyclesToMicroseconds(width * 21 + 16);
}