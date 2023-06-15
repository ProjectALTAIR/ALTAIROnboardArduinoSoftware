/**************************************************************************/
/*!
    @file     ALTAIR_TempSensor.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class of each temperature sensor in ALTAIR. 

    Justin Albert  jalbert@uvic.ca     began on 2 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_TempSensor_h
#define ALTAIR_TempSensor_h

#include "Arduino.h"

#define TEMP_SENSOR_COUNT                 4
#define TEMP_AVEREGING_WINDOW_SIZE        20

class ALTAIR_TempSensor {
  public:

    ALTAIR_TempSensor();
    //ALTAIR_TempSensor() :  _temp(         0.    )  {                       }

    float                   temp(               )  { return _temp        ; }
    float                   tempK(              )  { return _tempK       ; }
    int*                    tempWindow(         )  { return _tempWindow  ; }
    int                     tempWindowSum(      )  { return _tempWindowSum;}
    int                     analogTemp(         )  { return _analogTemp  ; }
    int                     analogInputPin(     )  { return _analogInputPin;}    
    //void                    setTemp( float temp )  {        _temp = temp ; }

    void                    initializeTempSensor(int analogInputPin);
    void                    storeAnalogTemp();
    void                    calculateTemp(      );
    unsigned short          packTemp(float theTemp);
    byte                    packTemp_byte(float theTemp);

  private:
    int                    _analogInputPin;
    int                    _tempWindow[TEMP_AVEREGING_WINDOW_SIZE];
    int                    _tempWindowSum;

    int                    _analogTemp                                   ;   // analog input
    float                  _temp                                         ;   // in degrees Celsius
    float                  _tempK                                        ;   // in Kelvin

};
#endif    //   ifndef ALTAIR_TempSensor_h
