/**************************************************************************/
/*!
    @file     ALTAIR_Battery.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for each of the ALTAIR main batteries.  ALTAIR
    nominally contains two 11.1 V LiPoly batteries of type
    ,
    and monitors their voltages via 1/3 voltage dividers connected to 
    an analog input pin for each battery.

    Justin Albert  jalbert@uvic.ca     began on 28 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef   ALTAIR_BATTERY_h
#define   ALTAIR_BATTERY_h

#include "Arduino.h"

#define   ALTAIR_GENOPSBAT_VMON_PIN                    A3              // *** FIX THESE !!!!! ***
#define   ALTAIR_PROPBAT_VMON_PIN                      A4              // *** FIX THESE !!!!! ***
#define   ALTAIRBAT_VOLTAGEDIVIDER                     (0.33)

class ALTAIR_Battery {
  public:

    ALTAIR_Battery(              byte   adcPin  ) : _adcPin( adcPin ) { }

    float           readVoltage(                )                     { return analogRead(_adcPin) * ALTAIRBAT_VOLTAGEDIVIDER ; }

  private:
    byte           _adcPin                       ;

};
#endif    //   ifndef ALTAIR_ARDUINOMICRO_h
