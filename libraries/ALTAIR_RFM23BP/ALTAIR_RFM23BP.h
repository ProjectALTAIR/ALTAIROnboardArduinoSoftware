/**************************************************************************/
/*!
    @file     ALTAIR_RFM23BP.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the telemetry interface class for the ALTAIR RFM23BP
    radio transceiver, which operates at 433 MHz.  This class derives 
    from the ALTAIR_GenTelInt generic telemetry interface base class.

    Justin Albert  jalbert@uvic.ca     began on 15 Oct. 2017

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/
#ifndef ALTAIR_RFM23BP_h
#define ALTAIR_RFM23BP_h

#include "ALTAIR_GenTelInt.h"
#include <RH_RF22.h>

class ALTAIR_RFM23BP : public ALTAIR_GenTelInt {
  public:
    bool    send(unsigned char aChar);
    bool    send(const uint8_t* aString);
    bool    sendAsIndivChars(const uint8_t* aString);
    bool    available();                       // if a byte is available for reading, returns true
    bool    isBusy();
    bool    initialize(const char* aString = "");
    byte    read();
    bool    readMessage(unsigned char* buffer, unsigned char* length);
    char    lastRSSI();                        // The RSSI value of most recently received message,
                                               // return value is in dBm, response of +127 means 
                                               // failed to get the last RSSI value.
    ALTAIR_RFM23BP(byte RFM23_chipselectpin, byte RFM23_interruptpin);

  protected:
    ALTAIR_RFM23BP();

  private:
    // this class is basically just a container for the RadioHead RH_RF22 class
    RH_RF22    _theRFM23BP;
  
};
#endif

