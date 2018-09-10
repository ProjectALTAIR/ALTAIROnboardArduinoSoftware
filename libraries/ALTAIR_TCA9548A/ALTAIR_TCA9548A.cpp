/**************************************************************************/
/*!
    @file     ALTAIR_TCA9548A.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR Adafruit TCA9548A 1-to-8 I2C
    multiplexer breakout board.  It is a static class, with one static
    member function (tcaselect), and thus should never be instantiated.

    Justin Albert  jalbert@uvic.ca     began on 9 Sep. 2018

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_TCA9548A.h"
#include <Wire.h>


/**************************************************************************/
/*!
 @brief  Constructor (never used).
*/
/**************************************************************************/
ALTAIR_TCA9548A::ALTAIR_TCA9548A()
{
}


/**************************************************************************/
/*!
 @brief  Select I2C TCA mux location
*/
/**************************************************************************/
void ALTAIR_TCA9548A::tcaselect(char tcaLocation) {

    if (tcaLocation > TCA9548A_MAXLOC || tcaLocation < TCA9548A_EVERYTHINGELSE ) return;
 
    Wire.beginTransmission( TCA9548A_I2CADDRESS     ) ;
    if ( tcaLocation ==     TCA9548A_EVERYTHINGELSE ) {
        Wire.write(         TCA9548A_MINLOC         ) ;
    } else                                            {
        Wire.write(         1 << tcaLocation        ) ;
    }
    Wire.endTransmission(                           ) ;  
}

