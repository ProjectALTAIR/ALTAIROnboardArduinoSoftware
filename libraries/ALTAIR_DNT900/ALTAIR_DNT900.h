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

#define  DEFAULT_DNT_SERIALID          1
#define  DEFAULT_DNTHWRESETPIN        27
#define  DEFAULT_DNTCTSPIN            28
#define  DEFAULT_DNTRTSPIN            29

class ALTAIR_DNT900 : public ALTAIR_GenTelInt {
  public:
    virtual bool    send(              unsigned char  aChar                                  );
    virtual bool    send(              const uint8_t* aString                                );
    virtual bool    sendAsIndivChars(  const uint8_t* aString                                );
    virtual bool    available();                              // If a byte is available for reading, returns true.
    virtual bool    isBusy();
    virtual bool    initialize(        const char*    aString       = ""                     );
    virtual byte    read();
    virtual char    lastRSSI();                               // The RSSI value of most recently received message,
                                                              // return value is in dBm, response of +127 means 
                                                              // failed to get the last RSSI value.
    ALTAIR_DNT900(const char serialID, const char     dntHwResetPin = DEFAULT_DNTHWRESETPIN, 
                                       const char     dntCTSPin     = DEFAULT_DNTCTSPIN, 
                                       const char     dntRTSPin     = DEFAULT_DNTRTSPIN      );
    ALTAIR_DNT900();                                          // No arguments => all default values.

  protected:

  private:
    char _serialID;
    char _dntHwResetPin;
    char _dntCTSPin;
    char _dntRTSPin;
  
};
#endif

