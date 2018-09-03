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

class ALTAIR_TempSensor {
  public:

    ALTAIR_TempSensor() :  _temp(         0.    )  {                       }

    float                   temp(               )  { return _temp        ; }
    void                    setTemp( float temp )  {        _temp = temp ; }

  private:
    float                  _temp                                         ;   // in degrees Celsius

};
#endif    //   ifndef ALTAIR_TempSensor_h
