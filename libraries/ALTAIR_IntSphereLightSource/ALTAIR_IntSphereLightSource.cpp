/**************************************************************************/
/*!
    @file     ALTAIR_IntSphereLightSource.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR integrating sphere-based light        
    source, including its four monochromatic light sources (three laser      
    diode modules, at 440 nm, 635 nm, and 670 nm, and one                  
    frequency-doubled Nd:YAG laser module at 532 nm).  It inherits from     
    the ALTAIR_LightSource base class.

    Justin Albert  jalbert@uvic.ca     began on 29 Aug. 2018

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_IntSphereLightSource.h"


/**************************************************************************/
/*!
 @brief  Constructor.  
*/
/**************************************************************************/
ALTAIR_IntSphereLightSource::ALTAIR_IntSphereLightSource(const char blue440nmLaserPin    ,
                                                         const char green532nmLaserPin   ,
                                                         const char red635nmLaserPin     ,
                                                         const char red670nmLaserPin   ) :
  _green532nmLaserPin(                                              green532nmLaserPin ) ,
  _red670nmLaserPin(                                                red670nmLaserPin   ) ,
  _red635nmLaserPin(                                                red635nmLaserPin   ) ,
  _blue440nmLaserPin(                                               blue440nmLaserPin  )
{

}

/**************************************************************************/
/*!
 @brief  Default constructor (unused).  
*/
/**************************************************************************/
ALTAIR_IntSphereLightSource::ALTAIR_IntSphereLightSource(                              ) :
  _green532nmLaserPin(                                     DEFAULT_GREEN532NMLASERPIN  ) ,
  _red670nmLaserPin(                                       DEFAULT_RED670NMLASERPIN    ) ,
  _red635nmLaserPin(                                       DEFAULT_RED635NMLASERPIN    ) ,
  _blue440nmLaserPin(                                      DEFAULT_BLUE440NMLASERPIN   )
{

}

/**************************************************************************/
/*!
 @brief  Initialize the light source control pins after power-on.
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::initialize(                                          )
{
  pinMode(_green532nmLaserPin, OUTPUT);
  pinMode(_red670nmLaserPin,   OUTPUT);
  pinMode(_red635nmLaserPin,   OUTPUT);
  pinMode(_blue440nmLaserPin,  OUTPUT);

  setInitialized(                    );

// normal situation: flash yellow LEDs then NO lights on (formerly it was yellow LEDs and green  
// laser on, but that heats up the I-drive transistor too much)
  setLightsNormal(                   );
}


/**************************************************************************/
/*!
 @brief  Set the light source output to its normal state.
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::setLightsNormal(                                     )
{
// Normal situation: turn off the lasers (and flash yellow LEDs then NO LEDs on -- formerly 
// it was yellow LEDs and green laser on, but that heats up the I-drive transistor too much).
  turnOffLasers(                 );
}

/**************************************************************************/
/*!
 @brief  Perform a test flash pattern when primary radio is transmitting:
         turn on the the blue laser (and the red LEDs).
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::setLightsPrimaryRadio(                               )
{
// Primary radio is transmitting: turn on the blue laser (and the red LEDs).
  turnOnBlueLaser();
}

/**************************************************************************/
/*!
 @brief  Perform a test flash pattern when a backup radio is transmitting:
         turn on the red 635 nm laser (and the blue LEDs).
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::setLightsBackupRadio(                                )
{
// A backup radio is transmitting: turn on the red 635 nm laser (and the blue LEDs).
  turnOnRed635nmLaser();
}

/**************************************************************************/
/*!
 @brief  Turn off all the laser light sources.
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::turnOffLasers(                                       )
{
  if (isInitialized(                       )) {
      digitalWrite(_green532nmLaserPin, LOW);   // turn this laser off by making the voltage LOW
      digitalWrite(_red670nmLaserPin,   LOW);   // turn this LD off (LOW is the voltage level)
      digitalWrite(_red635nmLaserPin,   LOW);   // turn this LD off (LOW is the voltage level)
      digitalWrite(_blue440nmLaserPin,  LOW);   // turn this LD off (LOW is the voltage level)   
      turnOff(                             );
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn on the blue laser (and only the blue laser).
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::turnOnBlueLaser(                                     )
{
  if (isInitialized(                        )) {
      turnOn(                               );
      digitalWrite(_green532nmLaserPin, LOW );  // turn this laser off by making the voltage LOW
      digitalWrite(_red670nmLaserPin,   LOW );  // turn this LD off (LOW is the voltage level)
      digitalWrite(_red635nmLaserPin,   LOW );  // turn this LD off (LOW is the voltage level)
      digitalWrite(_blue440nmLaserPin,  HIGH);  // turn this LD on (HIGH is the voltage level)   
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn on the red 635 nm laser (and only the red 635 nm laser).
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::turnOnRed635nmLaser(                                 )
{
  if (isInitialized(                        )) {
      turnOn(                               );
      digitalWrite(_green532nmLaserPin, LOW );  // turn this laser off by making the voltage LOW
      digitalWrite(_red670nmLaserPin,   LOW );  // turn this LD off (LOW is the voltage level)
      digitalWrite(_red635nmLaserPin,   HIGH);  // turn this LD on (HIGH is the voltage level)
      digitalWrite(_blue440nmLaserPin,  LOW );  // turn this LD off (LOW is the voltage level)   
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

