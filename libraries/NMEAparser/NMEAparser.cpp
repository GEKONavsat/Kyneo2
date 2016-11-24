/**************************************************************************************************
 * \file 	NMEAparser.cpp
 *
 * \brief	NMEA frames parsing functions.
 *
 * \author	GEKO NavSat S.L. <info@gekonavsat.com>
 * \date	04/11/2016
 * \version	2
 * \modifications: None (first version)
 *************************************************************************************************/ 
 
#include "NMEAparser.h"

/**
 * \fn <NMEAparser()>
 * \pre {None.}
 * \brief {Default Constructor. Default configuration is set.}
 */
NMEAparser::NMEAparser(){
  this->utime = 0;
  this->lat = NAN;
  this->lon = NAN;
  this->alt = 0;
  this->hdg = 0;
}

/**
 * \fn 		<getutime()>
 * \pre 	{Object has been filled before.}
 * \brief 	{Returns unix time.}
 * \param 	{none}
 * \return 	Unix time
 */
uint32_t NMEAparser::getutime(){
	return this->utime;
}

/**
 * \fn 		<gettime()>
 * \pre 	{Object has been filled before.}
 * \brief 	{Returns time in char format.}
 * \param 	{none}
 * \return 	void
 */
void NMEAparser::gettime(char *frame){
	strcpy(frame,this->_time);
}

/**
 * \fn 		<getdate()>
 * \pre 	{Object has been filled before.}
 * \brief 	{Returns date in char format.}
 * \param 	{none}
 * \return 	void
 */
void NMEAparser::getdate(char *frame){
	strcpy(frame,this->_date);
}

/**
 * \fn 		<getlat()>
 * \pre 	{Object has been filled before.}
 * \brief 	{Returns latitude.}
 * \param 	{none}
 * \return 	Latitude
 */
float NMEAparser::getlat(){
	return this->lat;
}

/**
 * \fn 		<getlon()>
 * \pre 	{Object has been filled before.}
 * \brief 	{Returns longitude.}
 * \param 	{none}
 * \return 	Longitude
 */
float NMEAparser::getlon(){
	return this->lon;
}

/**
 * \fn 		<getalt()>
 * \pre 	{Object has been filled before.}
 * \brief 	{Returns altitude.}
 * \param 	{none}
 * \return 	Altitude
 */
int16_t NMEAparser::getalt(){
	return this->alt;
}

/**
 * \fn 		<gethdg()>
 * \pre 	{Object has been filled before.}
 * \brief 	{Returns heading.}
 * \param 	{none}
 * \return 	Heading
 */
uint16_t NMEAparser::gethdg(){
	return this->hdg;
}

/**
 * \fn 		<NMEATimeToUnixTime()>
 * \pre 	{}
 * \brief 	{Calculates Unix time out of the time field.}
 * \param 	{Time field and Date field}
 * \return 	Unix Time
 */
uint32_t NMEAparser::NMEATimeToUnixTime(char* timeField, char* dateField)
{
  char part[3] = {'\0', '\0', '\0'};
  uint8_t hour, min, sec, day, mon, year;
  memcpy(part, timeField, 2);
  hour = (uint8_t) atoi(part);
  memcpy(part, timeField + 2, 2);
  min = (uint8_t) atoi(part);
  memcpy(part, timeField + 4, 2);
  sec = (uint8_t) atoi(part);
  memcpy(part, dateField, 2);
  day = (uint8_t) atoi(part);
  memcpy(part, dateField + 2, 2);
  mon = (uint8_t) atoi(part);
  memcpy(part, dateField + 4, 2);
  year = (uint8_t) atoi(part);
  if( (hour >= 24) || (min >= 60) || (sec >= 60) || (day > 31) || (mon > 12) || (year > 99) ){
    return 0;
  }
  //Unix time: seconds elapsed since 01/01/1970 00:00 GMT.
  day--;      //Days from 0 to 30
  mon--;      //Months from 0 (Jan) to 11 (Dec)
  if (year < 70){
    year += 30;     //Count also 1970-2000 period
  } else {
    year -= 70;     //Discount 1970 "offset".
  }

  uint16_t daysElapsed = 0;
  switch(mon){
    case 11:  daysElapsed += 30;  //Sum Nov days
    case 10:  daysElapsed += 31;  //Sum Oct days
    case 9:   daysElapsed += 30;  //Sum Sep days
    case 8:   daysElapsed += 31;  //Sum Ago days
    case 7:   daysElapsed += 31;  //Sum Jul days
    case 6:   daysElapsed += 30;  //Sum Jun days
    case 5:   daysElapsed += 31;  //Sum May days
    case 4:   daysElapsed += 30;  //Sum Apr days
    case 3:   daysElapsed += 31;  //Sum Mar days
    case 2: if( (year+2) % 4 == 0 ){
              daysElapsed += 29;  //Sum Feb days (leap year)
            } else{
              daysElapsed += 28;  //Sum Feb days (non-leap year)
            }
    case 1:   daysElapsed += 31;  //Sum Jan days
    case 0:   daysElapsed += day; //Sum only current day in the month.
              break;
    default:  return 0;
  }
  daysElapsed += 365*year + (year/4);
  return (uint32_t) (86400UL*daysElapsed + 3600UL*hour + 60UL*min + sec);
}

/**
 * \fn 		<NMEACoordToDegrees()>
 * \pre 	{}
 * \brief 	{Calculates degrees out of coordinates and hemisphere data.}
 * \param 	{Coor and hemisphere}
 * \return 	Degrees
 */
float NMEAparser::NMEACoordToDegrees(char* valueField, char hemisphere)
{
  char *dot = strchr(valueField, '.');
  if( (dot == NULL) || ((dot - valueField) < 4) ){
    return NAN;
  }
  int deg;
  deg = atoi(valueField) / 100;
  float coord = atof(dot - 2)/60 + deg;
  if( (hemisphere == 'N') || (hemisphere == 'E') ){
    return coord;
  }
  else if( (hemisphere == 'S') || (hemisphere == 'W') ){
    return -coord;
  }
  else{
    return NAN;
  }
}

/**
 * \fn 		<parseNMEA()>
 * \pre 	{}
 * \brief 	{Parses NMEA frame and fills the object parameters.}
 * \param 	{NMEA frame}
 * \return 	integer depending on the error case
 */
int NMEAparser::parseNMEA(char *nmea)
{
  char *cs = strchr(nmea, '*');
  size_t len = (cs - nmea);
  if( (cs == NULL) || (len == 0) || (len > MAX_NMEA_SIZE) || (*nmea++ != '$') ){
    //Wrong NMEA format: empty frame OR no checksum OR too long OR no $ header.
    return 1;
  }
  char fields_str[len];
  strncpy(fields_str, nmea, len - 1);
  fields_str[len - 1] = '\0';
  char *end = strchr(fields_str, ',');
  if( (end == NULL) || ((end - fields_str) != 5) ){
    return 2;
  }
  if(strncmp(end - 3, "RMC", 3) == 0) {
    //Serial.println("Parsing RMC...");
    return parseRMC(fields_str);
  }
  else if(strncmp(end - 3, "GGA", 3) == 0) {
    //Serial.println("Parsing GGA...");
    return parseGGA(fields_str);
  }
  else if(strncmp(end - 3, "GLL", 3) == 0) {
    //Serial.println("Parsing GLL...");
    return parseGLL(fields_str);
  }
  else{
    return 3; //Unsupported sentence.
  }
}

/**
 * \fn 		<parseRMC()>
 * \pre 	{}
 * \brief 	{Parses RMC frame and fills the object parameters.}
 * \param 	{RMC frame}
 * \return 	integer depending on the error case
 */
int NMEAparser::parseRMC(char *fields_str)
{
  char *coord, *timeField;   // Auxiliary variables.
  char *begin, *end;
  char fieldCount = 0;
  begin = fields_str;
  do{
    end = strchr(begin, ',');
    if (end != NULL){     //End points to a field separator.
      //Serial.println("Campo RMC");
      //Parse field.
      *end = '\0';
      switch(fieldCount){
        case 1:     //UTC time
          if( (end - begin) < 6 ){ return 2; }
          timeField = begin; // It will be parsed with date later
          shortString(timeField, this->_time, 6);
          break;
        case 3:     //Latitude
        case 5:     //Longitude
          if( (end - begin) < 9 ){ return 2; }
          coord = begin;
          break;
        case 4:     //North/South
          if( (end - begin) != 1 ){ return 2; }
          this->lat = NMEACoordToDegrees(coord, *begin);
          break;
        case 6:     //East/West
          if( (end - begin) != 1 ){ return 2; }
          this->lon = NMEACoordToDegrees(coord, *begin);
          break;
        case 8:     //Heading
          if( (end - begin) < 3 ){ return 2; }
          this->hdg = atoi(begin);
          break;
        case 9:     //UTC date
          if( (end - begin) != 6 ){ return 2; }
          strcpy(this->_date,begin);
          this->utime = NMEATimeToUnixTime(timeField, begin);
          break;
        case 0:     //GxRMC
        case 2:     //Valid
        case 7:     //Speed (Knots)
        case 10:    //Magnetic variation
        default:
          break;
      }
      fieldCount++;
      begin = end + 1;      //Skip separator and set begin for finding next token.
    }
    else{                 //Last field.
      //Parse last field. RMC: East/West of Magnetic Variation.
      break;
    }
  } while(begin != NULL);
  return 0;
}

/**
 * \fn 		<parseGGA()>
 * \pre 	{}
 * \brief 	{Parses GGA frame and fills the object parameters.}
 * \param 	{GGA frame}
 * \return 	integer depending on the error case
 */
int NMEAparser::parseGGA(char *fields_str)
{
  char *coord;          // Auxiliary variables.
  char *begin, *end;
  char fieldCount = 0;
  begin = fields_str;
  do{
    end = strchr(begin, ',');
    if (end != NULL){     //End points to a field separator.
      //Serial.println("Campo GGA");
      //Parse field.
      *end = '\0';
      switch(fieldCount){
        case 1:     //UTC time.
          shortString(begin, this->_time, 6);
	    case 2:     //Latitude
        case 4:     //Longitude
          if( (end - begin) < 9 ){ return 2; }
          coord = begin;
          break;
        case 3:     //North/South
          if( (end - begin) != 1 ){ return 2; }
          this->lat = NMEACoordToDegrees(coord, *begin);
          break;
        case 5:     //East/West
          if( (end - begin) != 1 ){ return 2; }
          this->lon = NMEACoordToDegrees(coord, *begin);
          break;
        case 9:     //Altitude above mean sea level in meters.
          if( (end - begin) < 3 ){ return 2; }
          this->alt = atoi(begin);
          break;
        case 8:		//Status deleted from original safetrails code
        case 0:     //GxGGA
        case 6:     //Fix Quality
        case 7:     //Number of satellites
        case 10:    //Height of geoid
        case 11:    //time in seconds since last DGPS update (generally empty)
        case 12:    //DGPS station ID (generally empty)
        default:
          break;
      }
      fieldCount++;
      begin = end + 1;      //Skip separator and set begin for finding next token.
    }
    else{                 //Last field.
      //Parse last field. RMC: East/West of Magnetic Variation.
      break;
    }
  } while(begin != NULL);
  return 0;
}

/**
 * \fn 		<parseGLL()>
 * \pre 	{}
 * \brief 	{Parses GLL frame and fills the object parameters.}
 * \param 	{GLL frame}
 * \return 	integer depending on the error case
 */
int NMEAparser::parseGLL(char *fields_str)
{
  char *coord, *timeField;   // Auxiliary variables.
  char *begin, *end;
  char fieldCount = 0;
  begin = fields_str;
  do{
    end = strchr(begin, ',');
    if (end != NULL){     //End points to a field separator.
      //Serial.println("Campo RMC");
      //Parse field.
      *end = '\0';
      switch(fieldCount){
		  case 1:		// Latitude
		  case 3:		// Longitude
          	if( (end - begin) < 9 ){ return 2; }
          	coord = begin;
          	break;
          case 2:		// North/South
          	if( (end - begin) != 1 ){ return 2; }
          	this->lat = NMEACoordToDegrees(coord, *begin);
          	break;
          case 4:		// East/West
          	if( (end - begin) != 1 ){ return 2; }
          	this->lon = NMEACoordToDegrees(coord, *begin);
         	break;
          default:
          	break;
      }
      fieldCount++;
      begin = end + 1;      //Skip separator and set begin for finding next token.
    }
    else{                 //Last field.
      //Parse last field. RMC: East/West of Magnetic Variation.
      break;
    }
  } while(begin != NULL);
  return 0;
}

/*---------------------------------------- Auxiliary Function ------------------------------*/
/**
 * \fn 		<shortString()>
 * \pre 	{}
 * \brief 	{Parses GLL frame and fills the object parameters.}
 * \param 	{GLL frame}
 * \return 	integer depending on the error case
 */
void NMEAparser::shortString(char *initframe, char *destframe, int lon){
  for(int i=0;i<lon;i++){
    destframe[i] = initframe[i];
  }
  destframe[lon] = (char)0;
}