/**************************************************************************/
/*!
    @file     ALTAIR_DNT900.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the telemetry interface class for the ALTAIR DNT900P
    radio transceiver, which operates at 910 MHz.  This class derives 
    from the ALTAIR_GenTelInt generic telemetry interface base class.

    Justin Albert  jalbert@uvic.ca     began on 15 Oct. 2017

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/
#ifndef ALTAIR_DNT900_h
#define ALTAIR_DNT900_h

#include "ALTAIR_GenTelInt.h"

class ALTAIR_DNT900 : public ALTAIR_GenTelInt {
  public:
    bool    send(unsigned char aChar);
    bool    send(const uint8_t* aString);
    bool    sendAsIndivChars(const uint8_t* aString);
    bool    available();                       // if a byte is available for reading, returns true
    bool    isBusy();
    bool    initialize(const char* aString = "");
    byte    read();
    char    lastRSSI();                        // The RSSI value of most recently received message,
                                               // return value is in dBm, response of +127 means 
                                               // failed to get the last RSSI value.
    ALTAIR_DNT900(const char serialID, const char dntHwResetPin, const char dntCTSPin, const char dntRTSPin);
  protected:
    ALTAIR_DNT900();

  private:

    char _serialID;
    char _dntHwResetPin;
    char _dntCTSPin;
    char _dntRTSPin;
  
};
#endif

