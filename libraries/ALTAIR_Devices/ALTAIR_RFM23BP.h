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

#define  DEFAULT_RFM_CHIPSELECTPIN    26
#define  DEFAULT_RFM_INTERRUPTPIN      2
#define  RFM23BP_RADIO_NAME     "RFM23BP"
#define  RFM_SPI_BYTE               0x00

class ALTAIR_RFM23BP : public ALTAIR_GenTelInt {
  public:
    virtual bool    send(              unsigned char  aChar                                  );
    virtual bool    send(              const uint8_t* aString                                );
    virtual bool    sendAsIndivChars(  const uint8_t* aString                                );
    virtual bool    available(                                                               ); // If a byte is available for reading, returns true.
    virtual bool    isBusy(                                                                  );
    virtual bool    initialize(        const char*    aString = ""                           );
    virtual byte    read(                                                                    );
    virtual bool    readMessage(       unsigned char* buffer, 
                                       unsigned char* length                                 );
    virtual const char*   radioName(                                                         );
    virtual radio_t radioType(                                                               );
    virtual char    lastRSSI(                                                                ); // The RSSI value of most recently received message,
                                                                                                // return value is in dBm, response of +127 means 
                                                                                                // failed to get the last RSSI value.
    ALTAIR_RFM23BP(                    byte           RFM23_chipselectpin, 
                                       byte           RFM23_interruptpin                     );
    ALTAIR_RFM23BP(                                                                          ); // No arguments => all default values.

  protected:

  private:
    // this class is basically just a container for the RadioHead RH_RF22 class
    RH_RF22    _theRFM23BP                                                                    ;
    byte       _RFM23_chipselectpin                                                           ;
  
};
#endif

