/**************************************************************************/
/*!
    @file     ALTAIR_TCA9548A.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR Adafruit TCA9548A 1-to-8 I2C 
    multiplexer breakout board.  It is a static class, with one static 
    member function (tcaselect), and thus should never be instantiated.

    Justin Albert  jalbert@uvic.ca     began on 9 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/
#ifndef ALTAIR_TCA9548A_h
#define ALTAIR_TCA9548A_h

#include "Arduino.h"

#define  TCA9548A_I2CADDRESS        0x70
#define  TCA9548A_MINLOC               0
#define  TCA9548A_MAXLOC               7
#define  TCA9548A_BME280PAYLOAD        0
#define  TCA9548A_EVERYTHINGELSE      -1

class ALTAIR_TCA9548A {
  public:
    static  void    tcaselect(      char           tcaLocation      );

  protected:
    ALTAIR_TCA9548A(                                                );    // This class shouldn't ever be instantiated.

  private:
  
};
#endif

