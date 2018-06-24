/**************************************************************************/
/*!
    @file     ALTAIR_GenTelInt.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the Generic Telemetry Interface base class for ALTAIR.
    Justin Albert  jalbert@uvic.ca     began on 8 Oct. 2017

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_GenTelInt.h"

#define  FAKE_RSSI_VAL   127

/**************************************************************************/
/*!
 @brief  Constructor.  Like all the methods in this base class, this will
         always be overridden (with the necessary transceiver-specific
         code) in the derived classes.
*/
/**************************************************************************/
ALTAIR_GenTelInt::ALTAIR_GenTelInt() {

}

/**************************************************************************/
/*!
 @brief  Initialize the transceiver after it is first powered on.  (The input
         string contains initialization information, if a non-default 
         initialization is wanted, for the specific transceiver.)
*/
/**************************************************************************/
bool ALTAIR_GenTelInt::initialize(const char* aString) {

    return false;

}

/**************************************************************************/
/*!
 @brief  Send one ASCII character.  (Returns false if send is unsuccessful.)
*/
/**************************************************************************/
bool ALTAIR_GenTelInt::send(unsigned char aChar) {

    return false;

}

/**************************************************************************/
/*!
 @brief  Send a string of ASCII characters.  (Returns false if send is 
         unsuccessful.)
*/
/**************************************************************************/
bool ALTAIR_GenTelInt::send(const uint8_t* aString) {

    return false;

}

/**************************************************************************/
/*!
 @brief  Send a string of ASCII characters, as a series of individual
         chars, individually in sequence from the first char to the null 
         char at the end of the string.  (Returns false if send is
         unsuccessful.)
*/
/**************************************************************************/
bool ALTAIR_GenTelInt::sendAsIndivChars(const uint8_t* aString) {

    return false;

}

/**************************************************************************/
/*!
 @brief  If a byte is currently available for reading, returns true.
         Otherwise, returns false.
*/
/**************************************************************************/
bool ALTAIR_GenTelInt::available() {

    return false;

}

/**************************************************************************/
/*!
 @brief  If the transceiver is currently busy, returns true.
         Otherwise, returns false.
*/
/**************************************************************************/
bool ALTAIR_GenTelInt::isBusy() {

    return false;

}


/**************************************************************************/
/*!
 @brief  Read a single ASCII character
*/
/**************************************************************************/
byte ALTAIR_GenTelInt::read() {

    return 0;

}

/**************************************************************************/
/*!
 @brief  Returns the most recent Return Signal Strength Information from
         the transceiver (i.e., from the most recent byte read).  The 
         return value is in dBm.  However, if a value of +127 is returned, 
         that means that the transceiver is not giving us an RSSI value
         (possibly because a byte hasn't been read from the transceiver yet 
         -- for example, see if perhaps data has only been written to, but   
         not yet read so far from the transceiver, if you get a +127).
*/
/**************************************************************************/
char ALTAIR_GenTelInt::lastRSSI() {

    return FAKE_RSSI_VAL;

}


