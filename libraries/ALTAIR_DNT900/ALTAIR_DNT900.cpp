/**************************************************************************/
/*!
    @file     ALTAIR_DNT900.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the telemetry interface class for the ALTAIR DNT900P
    radio transceiver, which operates at 910 MHz.  This class derives
    from the ALTAIR_GenTelInt generic telemetry interface base class.
    Justin Albert  jalbert@uvic.ca     began on 22 Oct. 2017

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_DNT900.h"

#define  DNT900_SERIAL_BAUDRATE    38400
#define  DNT_MAX_SEND_TRIES       500000

/**************************************************************************/
/*!
 @brief  Constructor.
*/
/**************************************************************************/
ALTAIR_DNT900::ALTAIR_DNT900(const char serialID, const char dntHwResetPin, const char dntCTSPin, const char dntRTSPin) :
   _serialID(serialID),
   _dntHwResetPin(dntHwResetPin),
   _dntCTSPin(dntCTSPin),
   _dntRTSPin(dntRTSPin)
{
}

/**************************************************************************/
/*!
 @brief  Default constructor.
*/
/**************************************************************************/
ALTAIR_DNT900::ALTAIR_DNT900() :
   _serialID(DEFAULT_DNT_SERIALID),
   _dntHwResetPin(DEFAULT_DNTHWRESETPIN),
   _dntCTSPin(DEFAULT_DNTCTSPIN),
   _dntRTSPin(DEFAULT_DNTRTSPIN)
{
}

/**************************************************************************/
/*!
 @brief  Initialize the transceiver after it is first powered on.  (The input
         string contains initialization information, if a non-default 
         initialization is wanted, for the specific transceiver.)
*/
/**************************************************************************/
bool ALTAIR_DNT900::initialize(const char* aString) {

    pinMode(                 _dntHwResetPin, OUTPUT);
    pinMode(                     _dntCTSPin, INPUT);
    pinMode(                     _dntRTSPin, OUTPUT);

    digitalWrite(            _dntHwResetPin, LOW);
    digitalWrite(                _dntRTSPin, HIGH);

    delay(200);

// END the hardware reset (i.e., set HwResetPin HIGH).  A Hw reset must occur every time the radio is powered up.
    digitalWrite(            _dntHwResetPin, HIGH);

  
    switch (_serialID) {
      case 0:
        Serial.begin(DNT900_SERIAL_BAUDRATE);
        break;
      case 1:
        Serial1.begin(DNT900_SERIAL_BAUDRATE);
        break;
      case 2:
        Serial2.begin(DNT900_SERIAL_BAUDRATE);
        break;
      case 3:
        Serial3.begin(DNT900_SERIAL_BAUDRATE);
        break;
      default:
        Serial.println(F("Unallowed serial ID provided in initialization of DNT900 radio transceiver!"));
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
bool ALTAIR_DNT900::send(unsigned char aChar) {

    int i = 0;
    while (digitalRead(_dntCTSPin) == HIGH && i < DNT_MAX_SEND_TRIES) {
        ++i;
    }
    if (i < DNT_MAX_SEND_TRIES) {
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
            Serial.println(F("Unallowed serial ID provided in initialization of DNT900 radio transceiver!"));
            while(1);
            return false;
        }
    } else {
        Serial.println(F("Unable to write to DNT: CTS pin is high"));
        return false;
    }

}

/**************************************************************************/
/*!
 @brief  Send a string of ASCII characters.  (Returns false if send is 
         unsuccessful.)
*/
/**************************************************************************/
bool ALTAIR_DNT900::send(const uint8_t* aString) {

    int i = 0;
    while (digitalRead(_dntCTSPin) == HIGH && i < DNT_MAX_SEND_TRIES) {
        ++i;
    }
    if (i < DNT_MAX_SEND_TRIES) {
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
            Serial.println(F("Unallowed serial ID provided in initialization of DNT900 radio transceiver!"));
            while(1);
            return false;
        }
    } else {
        Serial.println(F("Unable to write to DNT: CTS pin is high"));
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
bool ALTAIR_DNT900::sendAsIndivChars(const uint8_t* aString) {

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
bool ALTAIR_DNT900::available() {

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
        Serial.println(F("Unallowed serial ID provided in initialization of DNT900 radio transceiver!"));
        while(1);
        return false;
    }

}

/**************************************************************************/
/*!
 @brief  If the DNT transceiver is currently busy, returns true.
         Otherwise, returns false.
*/
/**************************************************************************/
bool ALTAIR_DNT900::isBusy() {

    return (digitalRead(_dntCTSPin) == HIGH);

}

/**************************************************************************/
/*!
 @brief  Read a single ASCII character
*/
/**************************************************************************/
byte ALTAIR_DNT900::read() {

    digitalWrite(_dntRTSPin, LOW);

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
        Serial.println(F("Unallowed serial ID provided in initialization of DNT900 radio transceiver!"));
        while(1);
        return 0;
    }

    digitalWrite(_dntRTSPin, HIGH);

}

/**************************************************************************/
/*!
 @brief  Return the name of the radio
*/
/**************************************************************************/
const char* ALTAIR_DNT900::radioName() {
    return DNT900_RADIO_NAME;
}

/**************************************************************************/
/*!
 @brief  Return the type of the radio
*/
/**************************************************************************/
radio_t ALTAIR_DNT900::radioType() {
    return dnt900;
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
char ALTAIR_DNT900::lastRSSI() {

// NEED TO IMPLEMENT THIS PROPERLY!!!
    return FAKE_RSSI_VAL;

}


