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

#ifndef  ALTAIR_GenTelInt_h
#define  ALTAIR_GenTelInt_h

#include "Arduino.h"

#define  FAKE_RSSI_VAL     127
#define  MAX_TERM_LENGTH   255
#define  MAX_READ_TRIES    100
#define  TX_START_BYTE    0xFA
#define  RX_START_BYTE    0xFC
#define  CALL_SIGN_STRING     " VE7XJA STATION ALTAIR "
#define  END_MESSAGE_STRING   " OVER "

typedef  enum { dnt900 ,
                shx144 ,
                rfm23bp } radio_t;

class    TinyGPSPlus;
class    ALTAIR_GlobalMotorControl;
class    ALTAIR_GlobalDeviceControl;
class    ALTAIR_GlobalLightControl;

class ALTAIR_GenTelInt {
  public:
    virtual bool         send(              unsigned char               aChar                   ) = 0;
    virtual bool         send(              const    uint8_t*           aString                 ) = 0;
            bool         sendGPS(           TinyGPSPlus&                gps                     )    ;
            bool         sendAllALTAIRInfo( TinyGPSPlus&                gps             ,
                                            ALTAIR_GlobalMotorControl&  motorControl    ,
                                            ALTAIR_GlobalDeviceControl& deviceControl   ,
                                            ALTAIR_GlobalLightControl&  lightControl            )    ;
            bool         sendCommandToALTAIR(        byte               commandByte1    ,
                                                     byte               commandByte2            )    ;  
    virtual bool         sendStart(                                                             ) { return send((unsigned char)  TX_START_BYTE      ) ; }
    virtual bool         sendAsIndivChars(  const    uint8_t*           aString                 ) = 0;
    virtual bool         sendCallSign(                                                          ) { return send((const uint8_t*) CALL_SIGN_STRING   ) ; }
    virtual bool         sendEndMessage(                                                        ) { return send((const uint8_t*) END_MESSAGE_STRING ) ; }
    virtual bool         available(                                                             ) = 0; // If a byte is available for reading, returns true.
    virtual bool         isBusy(                                                                ) = 0;
    virtual bool         initialize(        const    char*              aString         = ""    ) = 0;
    virtual byte         read(                                                                  ) = 0;
            byte*        readALTAIRInfo(             bool               isGroundStation = false )    ;
    virtual const char*  radioName(                                                             ) = 0;
    virtual radio_t      radioType(                                                             ) = 0;
    virtual char         lastRSSI(                                                              ) = 0; // The RSSI value of the most recently received 
                                                                                                       // message.  Return value is in dBm, response of 
                                                                                                       // +127 means failed to get the last RSSI value.
  protected:
            bool         sendBareGPS(                uint8_t            hour            ,
                                                     uint8_t            minute          ,
                                                     uint8_t            second          ,
                                                     int32_t            latitude        , 
                                                     int32_t            longitude       , 
                                                     int16_t            elevation               )    ;
            void         groundStationPrintRxInfo(   byte               term[]          ,
                                                     int                termLength              )    ;
    ALTAIR_GenTelInt(                                                                           )    ;

  private:
  
};
#endif

