/**************************************************************************/
/*!
    @file     ALTAIR_HMC5883L.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR HMC5883L magnetometer orientation 
    sensor, located inside the mast.  This class derives from the 
    ALTAIR_OrientSensor generic orientation sensor base class.

    Justin Albert  jalbert@uvic.ca     began on 7 Sep. 2018

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_HMC5883L.h"

/**************************************************************************/
/*!
 @brief  (Default) constructor.
*/
/**************************************************************************/
ALTAIR_HMC5883L::ALTAIR_HMC5883L(                              ) :
                _theHMC5883(          HMC5883L_SENSORID        )
{
}

/**************************************************************************/
/*!
 @brief  Initialize the orientation sensor during the setup routine.
*/
/**************************************************************************/
void ALTAIR_HMC5883L::initialize(                              ) 
{
  if(!_theHMC5883.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
}

/**************************************************************************/
/*!
 @brief  Return the heading, in degrees.
*/
/**************************************************************************/
float ALTAIR_HMC5883L::getHeading(                             )
{
    /* Get a new sensor event */
    _theHMC5883.getEvent(&_lastEvent);

    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    return      atan2(_lastEvent.magnetic.y, _lastEvent.magnetic.x) * 180. / M_PI;
}
