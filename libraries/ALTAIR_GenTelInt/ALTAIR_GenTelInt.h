/**************************************************************************/
/*!
    @file     ALTAIR_GenTelInt.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the Generic Telemetry Interface base class for ALTAIR.
    Note that the transceiver-specific classes (e.g. DNT900, SHX,
    RFM23BP, etc.) derive from this class.  Essentially all
    functionality (i.e. anything that isn't entirely specific to a 
    given transceiver) is abstracted to and available within this 
    interface.  Note that when necessary, inputs default to those
    for the DNT900. 
    Justin Albert  jalbert@uvic.ca     began on 8 Oct. 2017

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_GenTelInt_h
#define ALTAIR_GenTelInt_h

#include "Arduino.h"

class ALTAIR_GenTelInt {
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

  protected:
    ALTAIR_GenTelInt();


  private:
  
};
#endif

