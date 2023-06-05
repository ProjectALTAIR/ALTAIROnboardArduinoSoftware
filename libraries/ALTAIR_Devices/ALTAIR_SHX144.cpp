/**************************************************************************/
/*!
    @file     ALTAIR_SHX144.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the telemetry interface class for the ALTAIR SHX1-144-5
    radio transceiver, which operates at 144 MHz.  This class derives
    from the ALTAIR_GenTelInt generic telemetry interface base class.
    Justin Albert  jalbert@uvic.ca     began on 22 Oct. 2017

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_SHX144.h"
#include <SoftwareSerial.h>


/**************************************************************************/
/*!
 @brief  Constructor.
*/
/**************************************************************************/
ALTAIR_SHX144::ALTAIR_SHX144(const char serialID, const char fakeShxProgramRxPin, const char shxProgramPin, const char shxBusyPin) :
    _serialID(serialID) ,
    _fakeShxProgramRxPin(fakeShxProgramRxPin) ,
    _shxProgramPin(shxProgramPin) ,
    _shxBusyPin(shxBusyPin)
{
}

/**************************************************************************/
/*!
 @brief  Default constructor.
*/
/**************************************************************************/
ALTAIR_SHX144::ALTAIR_SHX144() :
    _serialID(DEFAULT_SHX_SERIALID) ,
    _fakeShxProgramRxPin(DEFAULT_FAKESHXPROGRAMRXPIN) ,
    _shxProgramPin(DEFAULT_SHXPROGRAMPIN) ,
    _shxBusyPin(DEFAULT_SHXBUSYPIN)
{
}

/**************************************************************************/
/*!
 @brief  Initialize the transceiver after it is first powered on.  (The input
         string contains initialization information, if a non-default 
         initialization is wanted, for the specific transceiver.)
*/
/**************************************************************************/
bool ALTAIR_SHX144::initialize(const char* aString) {

    SoftwareSerial shxProgramSerial(_fakeShxProgramRxPin, _shxProgramPin);

    pinMode(           _fakeShxProgramRxPin, INPUT);
    pinMode(                 _shxProgramPin, OUTPUT);
    pinMode(                    _shxBusyPin, INPUT);

    shxProgramSerial.begin(SHX144_PROGRAM_BAUDRATE);
  
//    delay(200);

    int bytesSent = 0;

    bytesSent += shxProgramSerial.write(byte('S'));
    bytesSent += shxProgramSerial.write(byte('E'));
    bytesSent += shxProgramSerial.write(byte('T'));
    bytesSent += shxProgramSerial.write(byte('M'));
    bytesSent += shxProgramSerial.write(byte('O'));
    bytesSent += shxProgramSerial.write(byte('D'));
    bytesSent += shxProgramSerial.write(byte('\r'));               // '\r' = ASCII 13 = carriage return

    if (bytesSent != 7) {
        Serial.println("Problem initializing SHX144");
        while(1);
    }

    delay(200);

    shxProgramSerial.end();

    digitalWrite(            _shxProgramPin, HIGH);  // The PGM pin is _active LOW_, so one must return it to its default
                                                     //  (i.e.: internal 47k pull-up to 4V) HIGH state in order to move
                                                     //  from programming mode to serial modem mode.

    switch (_serialID) {
      case 0:
        Serial.begin(SHX144_SERIAL_BAUDRATE);
        break;
      case 1:
        Serial1.begin(SHX144_SERIAL_BAUDRATE);
        break;
      case 2:
        Serial2.begin(SHX144_SERIAL_BAUDRATE);
        break;
      case 3:
        Serial3.begin(SHX144_SERIAL_BAUDRATE);
        break;
      default:
        Serial.println(F("Unallowed serial ID provided in initialization of SHX1 radio transceiver!"));
        while(1);
        break;
    }
    
    return true;

}



/**************************************************************************/
/*!
 @brief  Send one ASCII character.  (Returns false if send is unsuccessful.)
*/
/**************************************************************************/
bool ALTAIR_SHX144::send(unsigned char aChar) {

    switch (_serialID) {
      case 0:
        return Serial.write(aChar);
      case 1:
        return Serial1.write(aChar);
      case 2:
        return Serial2.write(aChar);
      case 3:
        return Serial3.write(aChar);
      default:
        Serial.println(F("Unallowed serial ID provided in initialization of SHX1 radio transceiver!"));
        while(1);
        return false;
    }

}

/**************************************************************************/
/*!
 @brief  Send a string of ASCII characters.  (Returns false if send is 
         unsuccessful.)
*/
/**************************************************************************/
bool ALTAIR_SHX144::send(const uint8_t* aString) {

    switch (_serialID) {
      case 0:
        return Serial.write((const char*) aString);
      case 1:
        return Serial1.write((const char*) aString);
      case 2:
        return Serial2.write((const char*) aString);
      case 3:
        return Serial3.write((const char*) aString);
      default:
        Serial.println(F("Unallowed serial ID provided in initialization of SHX1 radio transceiver!"));
        while(1);
        return false;
    }

}

/**************************************************************************/
/*!
 @brief  Send an array of bytes.  (Returns false if send is unsuccessful.)
*/
/**************************************************************************/
bool ALTAIR_SHX144::send(const uint8_t* anArray, const uint8_t arrayLen) {

    switch (_serialID) {
      case 0:
        return Serial.write(  anArray, arrayLen );
      case 1:
        return Serial1.write( anArray, arrayLen );
      case 2:
        return Serial2.write( anArray, arrayLen );
      case 3:
        return Serial3.write( anArray, arrayLen );
      default:
        Serial.println(F("Unallowed serial ID provided in initialization of SHX1 radio transceiver!"));
        while(1);
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
bool ALTAIR_SHX144::sendAsIndivChars(const uint8_t* aString) {

    switch (_serialID) {
      case 0:
        for (int i = 0; aString[i] != 0; ++i) {
            Serial.write(aString[i]);
        }
        return Serial.write(0);
      case 1:
        for (int i = 0; aString[i] != 0; ++i) {
            Serial1.write(aString[i]);
        }
        return Serial1.write(0);
      case 2:
        for (int i = 0; aString[i] != 0; ++i) {
            Serial2.write(aString[i]);
        }
        return Serial2.write(0);
      case 3:
        for (int i = 0; aString[i] != 0; ++i) {
            Serial3.write(aString[i]);
        }
        return Serial3.write(0);
      default:
        Serial.println(F("Unallowed serial ID provided in initialization of SHX1 radio transceiver!"));
        while(1);
        return false;
    }

}

/**************************************************************************/
/*!
 @brief  If a byte is currently available for reading, returns true.
         Otherwise, returns false.
*/
/**************************************************************************/
bool ALTAIR_SHX144::available() {

    switch (_serialID) {
      case 0:
        return Serial.available();
      case 1:
        return Serial1.available();
      case 2:
        return Serial2.available();
      case 3:
        return Serial3.available();
      default:
        Serial.println(F("Unallowed serial ID provided in initialization of SHX1 radio transceiver!"));
        while(1);
        return false;
    }

}

/**************************************************************************/
/*!
 @brief  If the SHX transceiver is currently busy, returns true.
         Otherwise, returns false.
*/
/**************************************************************************/
bool ALTAIR_SHX144::isBusy() {

    return (digitalRead(_shxBusyPin) == HIGH);

}

/**************************************************************************/
/*!
 @brief  Read a single ASCII character
*/
/**************************************************************************/
byte ALTAIR_SHX144::read() {

    switch (_serialID) {
      case 0:
        return Serial.read();
      case 1:
        return Serial1.read();
      case 2:
        return Serial2.read();
      case 3:
        return Serial3.read();
      default:
        Serial.println(F("Unallowed serial ID provided in initialization of SHX1 radio transceiver!"));
        while(1);
        return 0;
    }

}

/**************************************************************************/
/*!
 @brief  Return the name of the radio
*/
/**************************************************************************/
const char* ALTAIR_SHX144::radioName() {
    return SHX144_RADIO_NAME;
}

/**************************************************************************/
/*!
 @brief  Return the type of the radio
*/
/**************************************************************************/
radio_t ALTAIR_SHX144::radioType() {
    return shx144;
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
char ALTAIR_SHX144::lastRSSI() {

// NEED TO IMPLEMENT THIS PROPERLY!!!
    return FAKE_RSSI_VAL;

}
