/**************************************************************************/
/*!
    @file     ALTAIR_Battery.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for each of the ALTAIR main batteries.  ALTAIR
    nominally contains two 11.1 V LiPoly batteries, each of type
    ZIPPY Compact 2700mAh 3s 40c (found here:
      https://hobbyking.com/en_us/zippy-compact-2700mah-3s-40c-lipo-pack.html ) 
                             or
    Turnigy Nano-Tech 2650mah 3S 35~70C (similar to this:
      https://hobbyking.com/en_us/turnigy-battery-nano-tech-2650mah-3s-25-50c-lipo-pack-xt-60.html
      or this
      https://hobbyking.com/en_us/turnigy-nano-tech-2650mah-3s-30c-lipo-pack-wxt60.html )
    , and monitors their voltages via 1/3 voltage dividers connected to 
    an analog input pin for each battery.

    Justin Albert  jalbert@uvic.ca     began on 28 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef   ALTAIR_BATTERY_h
#define   ALTAIR_BATTERY_h

#include "Arduino.h"

#define   ALTAIR_GENOPSBAT_VMON_PIN                    A3 
#define   ALTAIR_PROPBAT_VMON_PIN                      A4  
#define   ALTAIRBAT_VOLTAGEDIVIDER                     (0.33)

class ALTAIR_Battery {
  public:

    ALTAIR_Battery(              byte   adcPin  ) : _adcPin( adcPin ) { }

    float           readVoltage(                )                     { return analogRead(_adcPin) * ALTAIRBAT_VOLTAGEDIVIDER ; }

  private:
    byte           _adcPin                       ;

};
#endif    //   ifndef ALTAIR_ARDUINOMICRO_h
