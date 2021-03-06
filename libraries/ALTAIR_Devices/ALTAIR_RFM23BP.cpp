/**************************************************************************/
/*!
    @file     ALTAIR_RFM23BP.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the telemetry interface class for the ALTAIR RFM23BP
    radio transceiver, which operates at 433 MHz.  This class derives
    from the ALTAIR_GenTelInt generic telemetry interface base class.
    This particular class is just a container class for, and thus just
    a wrapper around, the RadioHead RH_RF22 class.
    Justin Albert  jalbert@uvic.ca     began on 23 Oct. 2017

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_RFM23BP.h"

/**************************************************************************/
/*!
 @brief  Constructor.
*/
/**************************************************************************/
ALTAIR_RFM23BP::ALTAIR_RFM23BP(byte RFM23_chipselectpin  , byte RFM23_interruptpin  ) :
                   _theRFM23BP(     RFM23_chipselectpin  ,      RFM23_interruptpin  ) ,
          _RFM23_chipselectpin(     RFM23_chipselectpin                             ) ,
          _lastSentString2(         false                                           )
{
}

/**************************************************************************/
/*!
 @brief  Default constructor.
*/
/**************************************************************************/
ALTAIR_RFM23BP::ALTAIR_RFM23BP(                                                     ) :
                   _theRFM23BP( DEFAULT_RFM_CHIPSELECTPIN, DEFAULT_RFM_INTERRUPTPIN ) ,
          _RFM23_chipselectpin( DEFAULT_RFM_CHIPSELECTPIN                           ) ,
          _lastSentString2(     false                                               )
{
}

/**************************************************************************/
/*!
 @brief  Initialize the transceiver after it is first powered on.  (The input
         string contains initialization information, if a non-default 
         initialization is wanted for the specific transceiver.)
*/
/**************************************************************************/
bool ALTAIR_RFM23BP::initialize(const char* aString) {

    pinMode(               _RFM23_chipselectpin ,  OUTPUT       )   ;
    bool   initBool      = _theRFM23BP.init(                    )   ;
    digitalWrite(          _RFM23_chipselectpin ,  LOW          )   ;   // try adding this
    byte   received_byte =  SPI.transfer(          RFM_SPI_BYTE )   ;   // try adding this
    digitalWrite(          _RFM23_chipselectpin ,  HIGH         )   ;   // try adding this
    return initBool                                                 ;
}

/**************************************************************************/
/*!
 @brief  Send one ASCII character.  (Returns false if send is unsuccessful.)
*/
/**************************************************************************/
bool ALTAIR_RFM23BP::send(unsigned char aChar) {

//    Don't null-terminate.
//    const unsigned char aCharStr[2] = { aChar, '\0' };
//    return _theRFM23BP.send(aCharStr, 2);
    const unsigned char aCharStr[1] = { aChar };
    return _theRFM23BP.send(aCharStr, 1);

}

/**************************************************************************/
/*!
 @brief  Send a string of ASCII characters.  (Returns false if send is 
         unsuccessful.)
*/
/**************************************************************************/
bool ALTAIR_RFM23BP::send(const uint8_t* aString) {

//    unsigned char stringLen = sizeof(aString);  // DON'T use sizeof here -- the result will always be 2!
    int stringLen = 0;
    while (aString[stringLen] != 0 && stringLen <= _theRFM23BP.maxMessageLength()) stringLen++;
    stringLen++;   // to get the null character at end
    if (stringLen <= _theRFM23BP.maxMessageLength(                 )) {
        _theRFM23BP.send(       aString,               stringLen    )   ;
        _theRFM23BP.waitPacketSent(                                 )   ;
        digitalWrite(          _RFM23_chipselectpin ,  LOW          )   ;   // try adding this
        byte   received_byte =  SPI.transfer(          RFM_SPI_BYTE )   ;   // try adding this
        digitalWrite(          _RFM23_chipselectpin ,  HIGH         )   ;   // try adding this
        return true;
    } else {
        return false;
    }

}

/**************************************************************************/
/*!
 @brief  Send an array of bytes.  (Returns false if send is unsuccessful.)
*/
/**************************************************************************/
bool ALTAIR_RFM23BP::send(const uint8_t* anArray, const uint8_t arrayLen) {

    if (arrayLen <= _theRFM23BP.maxMessageLength(                  )) {
        _theRFM23BP.send(       anArray,               arrayLen     )   ;
        _theRFM23BP.waitPacketSent(                                 )   ;
        digitalWrite(          _RFM23_chipselectpin ,  LOW          )   ;   // try adding this
        byte   received_byte =  SPI.transfer(          RFM_SPI_BYTE )   ;   // try adding this
        digitalWrite(          _RFM23_chipselectpin ,  HIGH         )   ;   // try adding this
        return true;
    } else {
        return false;
    }

}

/**************************************************************************/
/*!
 @brief  Send a string of ASCII characters, as a series of individual
         chars, individually in sequence from the first char to the null
         char at the end of the string.  (Returns false if send is
         unsuccessful.
*/
/**************************************************************************/
bool ALTAIR_RFM23BP::sendAsIndivChars(const unsigned char* aString) {

    for (int i = 0; aString[i] != 0; ++i) {
        send(aString[i]);
    }
    return send((unsigned char) '\0');

}

/**************************************************************************/
/*!
 @brief  If a byte is currently available for reading, returns true.
         Otherwise, returns false.
*/
/**************************************************************************/
bool ALTAIR_RFM23BP::available() {

    return _theRFM23BP.available();

}

/**************************************************************************/
/*!
 @brief  If the SHX transceiver is currently busy, returns true.
         Otherwise, returns false.
*/
/**************************************************************************/
bool ALTAIR_RFM23BP::isBusy() {

    return (!_theRFM23BP.waitCAD());

}

/**************************************************************************/
/*!
 @brief  Read a single ASCII character
*/
/**************************************************************************/
byte ALTAIR_RFM23BP::read() {

   unsigned char buffer[RH_RF22_FIFO_SIZE];
   unsigned char length;
   bool          messageRecvd = _theRFM23BP.recv(buffer, &length);
   if (messageRecvd) {
       return buffer[0];
   } else {
       return 0;
   }

}

/**************************************************************************/
/*!
 @brief  Read the full message received
*/
/**************************************************************************/
bool ALTAIR_RFM23BP::readMessage(unsigned char* buffer, unsigned char* length) {

   return _theRFM23BP.recv(buffer, length);

}

/**************************************************************************/
/*!
 @brief  Read a command sent up to ALTAIR from a ground station, or data
         sent down from ALTAIR to a ground station.
*/
/**************************************************************************/
void ALTAIR_RFM23BP::readALTAIRInfo(  byte command[],  bool isGroundStation )
{
    byte        buffer[MAX_TERM_LENGTH]                   ;
    byte        bufferLength                              ;
    byte        term[MAX_TERM_LENGTH]    =             "" ;
    int         termIndex                =              0 ;
    int         termLength               =              0 ;
    long        readTry                  =              0 ;
    byte        startByte                =  RX_START_BYTE ;
    if (isGroundStation)  startByte      =  TX_START_BYTE ;
                command[0]               =              0 ;
                command[1]               =              0 ;
    while (true) {
//        Serial.println(F("here1"));
        while (!available() && readTry < MAX_READ_TRIES) {
          ++readTry;
           delay(5);
        }
        if (readTry < MAX_READ_TRIES) {
//            Serial.print(F("here2: ")); Serial.println(readTry);
            bool readSuccess = readMessage(buffer, &bufferLength);
            if (readSuccess && (buffer[0] == startByte) && (buffer[1] < MAX_TERM_LENGTH)) {
//              Serial.println(F("here3"));
              termLength = ((int) (buffer[1]));
              for (termIndex = 0; termIndex < termLength; ++termIndex) {
                term[termIndex] = buffer[termIndex+2];
//                if (isGroundStation) Serial.println(term[termIndex], HEX);
              }
//              Serial.print(F("term[0] = ")); Serial.print(term[0], HEX); Serial.print(F("  term[1] = ")); Serial.println(term[1], HEX);
              command[0] = term[0];
              command[1] = term[1];
              if (isGroundStation) groundStationPrintRxInfo(term, termLength);
              break;
            }
        } else {
//          Serial.println(F("Radio is not available for reading"));
          break;
        }
    } 
    return;
}


/**************************************************************************/
/*!
 @brief  Return the name of the radio
*/
/**************************************************************************/
const char* ALTAIR_RFM23BP::radioName() {
    return RFM23BP_RADIO_NAME;
}

/**************************************************************************/
/*!
 @brief  Return the type of the radio
*/
/**************************************************************************/
radio_t ALTAIR_RFM23BP::radioType() {
    return rfm23bp;
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
char ALTAIR_RFM23BP::lastRSSI() {

    return _theRFM23BP.lastRssi();

}

/**************************************************************************/
/*!
 @brief  Did we last send string2? (Or string1?)
*/
/**************************************************************************/
bool ALTAIR_RFM23BP::lastSentString2() {
           _lastSentString2 = !_lastSentString2;
    return _lastSentString2;
}

