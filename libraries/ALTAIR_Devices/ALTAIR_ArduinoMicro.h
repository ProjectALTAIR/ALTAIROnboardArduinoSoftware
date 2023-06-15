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
#include "Wire.h"

#define   RPM_SENSOR_COUNT                        2
#define   CURRENT_SENSOR_COUNT                    2
#define   TEMP_SENSOR_COUNT                       4

#define   ARDUINOMICRO_I2CADDRESS                 0x08
#define   ARDUINOMICRO_DATABYTES                  16


class ALTAIR_ArduinoMicro {
  public:

    ALTAIR_ArduinoMicro(                                         )    ;

    virtual  void        initialize(                             )    {                         }
    virtual  void        getDataAfterInterval(    long interval  )    ;

             unsigned short*       packedRPM(                              )    { return _packedRPM     ; }
             unsigned short*       packedCurrent(                          )    { return _packedCurrent ; }
             unsigned short*       packedTemp(                             )    { return _packedTemp    ; }
             byte*       packedRPS_byte(                              )    { return _packedRPS_byte     ; }
             byte*       packedCurrent_byte(                          )    { return _packedCurrent_byte ; }
             byte*       packedTemp_byte(                             )    { return _packedTemp_byte    ; }
    
  private:

    unsigned long       _dataLastObtainedAtMillis                     ;
             unsigned short       _packedRPM[RPM_SENSOR_COUNT]                                 ;
             unsigned short       _packedCurrent[CURRENT_SENSOR_COUNT]                             ;
             unsigned short       _packedTemp[TEMP_SENSOR_COUNT]                                ;
             byte                 _packedRPS_byte[RPM_SENSOR_COUNT]                                 ;
             byte                 _packedCurrent_byte[CURRENT_SENSOR_COUNT]                             ;
             byte                 _packedTemp_byte[TEMP_SENSOR_COUNT]                                ;
             int        RPM[RPM_SENSOR_COUNT];
             float      Current[CURRENT_SENSOR_COUNT];
             float      Temp[TEMP_SENSOR_COUNT];
};
#endif    //   ifndef ALTAIR_ARDUINOMICRO_h
