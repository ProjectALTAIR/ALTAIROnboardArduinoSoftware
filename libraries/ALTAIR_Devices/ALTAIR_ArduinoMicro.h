/**************************************************************************/
/*!
    @file     ALTAIR_ArduinoMicro.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the Arduino Micro I2C slave board, which reads 
    and controls the 4 propulsion system RPM sensors, the 4 propulsion 
    system current sensors, and the 8 propulsion system temp sensors.

    Justin Albert  jalbert@uvic.ca     began on 18 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef   ALTAIR_ARDUINOMICRO_h
#define   ALTAIR_ARDUINOMICRO_h

#include "Arduino.h"

#define   ARDUINOMICRO_I2CADDRESS                 0x08
#define   ARDUINOMICRO_DATABYTES                    16

class ALTAIR_ArduinoMicro {
  public:

    ALTAIR_ArduinoMicro(                                         )    ;

    virtual  void        initialize(                             )    {                         }
    virtual  void        getDataAfterInterval(    long interval  )    ;

             byte*       packedRPM(                              )    { return _packedRPM     ; }
             byte*       packedCurrent(                          )    { return _packedCurrent ; }
             byte*       packedTemp(                             )    { return _packedTemp    ; }
    
  private:

    unsigned long       _dataLastObtainedAtMillis                     ;
             byte       _packedRPM[4]                                 ;
             byte       _packedCurrent[4]                             ;
             byte       _packedTemp[8]                                ;
};
#endif    //   ifndef ALTAIR_ARDUINOMICRO_h
