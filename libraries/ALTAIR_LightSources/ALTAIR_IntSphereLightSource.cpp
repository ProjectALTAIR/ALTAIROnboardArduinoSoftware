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
  _blue440nmLaserPin(                                               blue440nmLaserPin  ) ,
  _green532nmLaserState(                                            LOW                ) ,
  _red670nmLaserState(                                              LOW                ) ,
  _red635nmLaserState(                                              LOW                ) ,
  _blue440nmLaserState(                                             LOW                )
{
}

/**************************************************************************/
/*!
 @brief  Default constructor.  
*/
/**************************************************************************/
ALTAIR_IntSphereLightSource::ALTAIR_IntSphereLightSource(                              ) :
  _green532nmLaserPin(                                     DEFAULT_GREEN532NMLASERPIN  ) ,
  _red670nmLaserPin(                                       DEFAULT_RED670NMLASERPIN    ) ,
  _red635nmLaserPin(                                       DEFAULT_RED635NMLASERPIN    ) ,
  _blue440nmLaserPin(                                      DEFAULT_BLUE440NMLASERPIN   ) ,
  _green532nmLaserState(                                   LOW                         ) ,
  _red670nmLaserState(                                     LOW                         ) ,
  _red635nmLaserState(                                     LOW                         ) ,
  _blue440nmLaserState(                                    LOW                         )
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

  resetLights(                       );
}


/**************************************************************************/
/*!
 @brief  Set the light source output to its steady state.
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::resetLights(                                         )
{
  if (isInitialized(                       )) {
      digitalWrite(_green532nmLaserPin, _green532nmLaserState );
      digitalWrite(_red670nmLaserPin,   _red670nmLaserState   );
      digitalWrite(_red635nmLaserPin,   _red635nmLaserState   );
      digitalWrite(_blue440nmLaserPin,  _blue440nmLaserState  );
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Return a uint8_t with its lowest 4 bits set according to the states
         of the 4 lasers
*/
/**************************************************************************/
uint8_t ALTAIR_IntSphereLightSource::getStatusNibble(                                  )
{
      uint8_t blue440nmLaserBit  = _blue440nmLaserState  == HIGH ? 1 : 0;
      uint8_t green532nmLaserBit = _green532nmLaserState == HIGH ? 2 : 0;
      uint8_t red635nmLaserBit   = _red635nmLaserState   == HIGH ? 4 : 0;
      uint8_t red670nmLaserBit   = _red670nmLaserState   == HIGH ? 8 : 0;
      return (red670nmLaserBit | red635nmLaserBit | green532nmLaserBit | blue440nmLaserBit);
}

/**************************************************************************/
/*!
 @brief  Perform a test flash pattern when primary radio is transmitting:
         turn on the the blue laser (and the red LEDs).
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::setLightsPrimaryRadio(                               )
{
// Primary radio is transmitting: temporarily turn on the blue laser (and the red LEDs).
  flashBlueLaser();
}

/**************************************************************************/
/*!
 @brief  Perform a test flash pattern when a backup radio is transmitting:
         turn on the red 635 nm laser (and the blue LEDs).
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::setLightsBackupRadio(                                )
{
// A backup radio is transmitting: temporarily turn on the red 635 nm laser (and the blue LEDs).
  flashRed635nmLaser();
}

/**************************************************************************/
/*!
 @brief  Turn off all the laser light sources.
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::turnOffLasers(                                       )
{
  if (isInitialized(                       )) {
     _green532nmLaserState    = LOW ;
     _red670nmLaserState      = LOW ;
     _red635nmLaserState      = LOW ;
     _blue440nmLaserState     = LOW ;
      resetLights();    
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Flash the blue laser (and only the blue laser).
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::flashBlueLaser(                                     )
{
  if (isInitialized(                        )) {
      digitalWrite(_blue440nmLaserPin,  !_blue440nmLaserState); 
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Flash the red 635 nm laser (and only the red 635 nm laser).
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::flashRed635nmLaser(                                 )
{
  if (isInitialized(                        )) {
      digitalWrite(_red635nmLaserPin,  !_red635nmLaserState); 
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn on the blue laser (and only the blue laser).
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::turnOnBlueLaser(           )
{
  if (isInitialized(                                        )) {
     _blue440nmLaserState              = HIGH                 ;  // turn this laser on (HIGH is the voltage level)
      digitalWrite(_blue440nmLaserPin,  _blue440nmLaserState ); 
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn off the blue laser (and only the blue laser).
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::turnOffBlueLaser(          )
{
  if (isInitialized(                                        )) {
     _blue440nmLaserState              = LOW                  ;  // turn this laser off (LOW is the voltage level)
      digitalWrite(_blue440nmLaserPin,  _blue440nmLaserState ); 
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn on the green laser (and only the green laser).
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::turnOnGreenLaser(          )
{
  if (isInitialized(                                        )) {
     _green532nmLaserState             = HIGH                 ;  // turn this laser on (HIGH is the voltage level)
      digitalWrite(_green532nmLaserPin, _green532nmLaserState); 
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn off the green laser (and only the green laser).
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::turnOffGreenLaser(         )
{
  if (isInitialized(                                        )) {
     _green532nmLaserState             = LOW                  ;  // turn this laser off (LOW is the voltage level)
      digitalWrite(_green532nmLaserPin, _green532nmLaserState); 
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn on the red 635 nm laser (and only the red 635 nm laser).
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::turnOnRed635nmLaser(       )
{
  if (isInitialized(                                        )) {
     _red635nmLaserState               = HIGH                 ;  // turn this laser on (HIGH is the voltage level)
      digitalWrite(_red635nmLaserPin,   _red635nmLaserState  ); 
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn off the red 635 nm laser (and only the red 635 nm laser).
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::turnOffRed635nmLaser(      )
{
  if (isInitialized(                                        )) {
     _red635nmLaserState               = LOW                  ;  // turn this laser off (LOW is the voltage level)
      digitalWrite(_red635nmLaserPin,   _red635nmLaserState  ); 
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn on the red 670 nm laser (and only the red 670 nm laser).
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::turnOnRed670nmLaser(       )
{
  if (isInitialized(                                        )) {
     _red670nmLaserState               = HIGH                 ;  // turn this laser on (HIGH is the voltage level)
      digitalWrite(_red670nmLaserPin,   _red670nmLaserState  ); 
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn off the red 670 nm laser (and only the red 670 nm laser).
*/
/**************************************************************************/
void ALTAIR_IntSphereLightSource::turnOffRed670nmLaser(      )
{
  if (isInitialized(                                        )) {
     _red670nmLaserState               = LOW                  ;  // turn this laser off (LOW is the voltage level)
      digitalWrite(_red670nmLaserPin,   _red670nmLaserState  ); 
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

