/**************************************************************************/
/*!
    @file     ALTAIR_LightSource.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the base class for ALTAIR light sources, including the
    integrating sphere-based light source containing the laser diode
    light sources, the diffusive light source containing the LED light
    sources, as well as any other sources (e.g. microwave polarimetric) 
    that may be added in the future.
    Note that the light source-specific classes (e.g.
    IntSphereLightSource, DiffLEDLightSource, etc.) derive from this
    class.  Essentially all functionality (i.e. anything that isn't
    entirely specific to a given light source) is abstracted to and
    available within this interface.  Note that when necessary, inputs
    default to those for the integrating sphere-based light source.

    Justin Albert  jalbert@uvic.ca     began on 29 Aug. 2018

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_LightSource.h"


/**************************************************************************/
/*!
 @brief  Constructor.  This will always be overridden (with the necessary 
         light source-specific code) in the derived classes.
*/
/**************************************************************************/
ALTAIR_LightSource::ALTAIR_LightSource() :
  _isInitialized( false                ) ,
  _isOn(          false                )
{

}
