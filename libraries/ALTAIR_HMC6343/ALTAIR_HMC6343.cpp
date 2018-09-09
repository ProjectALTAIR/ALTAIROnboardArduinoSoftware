/**************************************************************************/
/*!
    @file     ALTAIR_HMC6343.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR Sparkfun HMC6343 orientation sensor,
    located on the mast.  This class derives from the ALTAIR_OrientSensor
    generic orientation sensor base class.

    Justin Albert  jalbert@uvic.ca     began on 7 Sep. 2018

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_HMC6343.h"

/**************************************************************************/
/*!
 @brief  (Default) constructor.
*/
/**************************************************************************/
ALTAIR_HMC6343::ALTAIR_HMC6343(                               )
{
}

/**************************************************************************/
/*!
 @brief  Initialize the orientation sensor during the setup routine.
*/
/**************************************************************************/
void ALTAIR_HMC6343::initialize(                              ) 
{
    Serial.println(F("HMC6343 magnetometer initialization..."));

    if (!_theHMC6343.init()) {
        Serial.println("SparkFun HMC6343 magnetometer initialization failed\n\r"); // Report failure, is the sensor wiring correct?
        while(1);
    } 
}
