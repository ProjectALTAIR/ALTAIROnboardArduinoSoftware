/**************************************************************************/
/*!
    @file     ALTAIR_DiffLEDLightSource.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR diffusive LED light source,
    including its four LED light sources (blue, green, yellow, and red).
    It inherits from the ALTAIR_LightSource base class.

    Justin Albert  jalbert@uvic.ca     began on 29 Aug. 2018

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_DiffLEDLightSource.h"


/**************************************************************************/
/*!
 @brief  Constructor.  
*/
/**************************************************************************/
ALTAIR_DiffLEDLightSource::ALTAIR_DiffLEDLightSource(const char blueLEDsPin     ,
                                                     const char greenLEDsPin    ,
                                                     const char yellowLEDsPin   ,
                                                     const char redLEDsPin    ) :
  _yellowLEDsPin(                                               yellowLEDsPin ) ,
  _redLEDsPin(                                                  redLEDsPin    ) ,
  _blueLEDsPin(                                                 blueLEDsPin   ) ,
  _greenLEDsPin(                                                greenLEDsPin  ) ,
  _yellowLEDsState(                                             LOW           ) ,
  _redLEDsState(                                                LOW           ) ,
  _blueLEDsState(                                               LOW           ) ,
  _greenLEDsState(                                              LOW           )
{
}

/**************************************************************************/
/*!
 @brief  Default constructor.  
*/
/**************************************************************************/
ALTAIR_DiffLEDLightSource::ALTAIR_DiffLEDLightSource(                          ) :
  _yellowLEDsPin(                                     DEFAULT_YELLOWLEDSPIN    ) ,
  _redLEDsPin(                                        DEFAULT_REDLEDSPIN       ) ,
  _blueLEDsPin(                                       DEFAULT_BLUELEDSPIN      ) ,
  _greenLEDsPin(                                      DEFAULT_GREENLEDSPIN     ) ,
  _yellowLEDsState(                                   LOW                      ) ,
  _redLEDsState(                                      LOW                      ) ,
  _blueLEDsState(                                     LOW                      ) ,
  _greenLEDsState(                                    LOW                      )
{
}

/**************************************************************************/
/*!
 @brief  Initialize the light source control pins after power-on.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::initialize(                                   )
{
  pinMode(_yellowLEDsPin, OUTPUT);
  pinMode(_redLEDsPin,    OUTPUT);
  pinMode(_blueLEDsPin,   OUTPUT);
  pinMode(_greenLEDsPin,  OUTPUT);

  setInitialized(               );

// Normal situation: flash yellow LEDs then NO lights on (formerly it was yellow LEDs and green  
// laser on, but that heats up the I-drive transistor too much).
  flashYellowLEDs(              );
  delay(                  20    );
  resetLights(                  );
}


/**************************************************************************/
/*!
 @brief  Set the light source output to its steady state.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::resetLights(                              )
{
  if (isInitialized(                   )) {
      digitalWrite(_yellowLEDsPin, _yellowLEDsState ); 
      digitalWrite(_redLEDsPin,    _redLEDsState    ); 
      digitalWrite(_blueLEDsPin,   _blueLEDsState   ); 
      digitalWrite(_greenLEDsPin,  _greenLEDsState  ); 
  }
}

/**************************************************************************/
/*!
 @brief  Perform a test flash pattern when primary radio is transmitting:
         turn on the red LEDs (and the blue laser)
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::setLightsPrimaryRadio(                        )
{
// Primary radio is transmitting: turn on the red LEDs (and the blue laser).
  flashRedLEDs();
}

/**************************************************************************/
/*!
 @brief  Perform a test flash pattern when a backup radio is transmitting:
         turn on the blue LEDs (and the red 635 nm laser)
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::setLightsBackupRadio(                         )
{
// A backup radio is transmitting: turn on the blue LEDs (and the red 635 nm laser).
  flashBlueLEDs();
}

/**************************************************************************/
/*!
 @brief  Turn off all the LED light sources.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::turnOffLEDs(                                  )
{
  if (isInitialized(                  )) {
     _yellowLEDsState = LOW ;   // turn these LEDs off (LOW is the voltage level)
     _redLEDsState    = LOW ;   // turn these LEDs off (LOW is the voltage level)
     _blueLEDsState   = LOW ;   // turn these LEDs off (LOW is the voltage level)
     _greenLEDsState  = LOW ;   // turn these LEDs off (LOW is the voltage level)   

      resetLights()         ;
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Flash yellow LED light sources.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::flashYellowLEDs(                   )
{
  if (isInitialized(                   )) {
      digitalWrite(_yellowLEDsPin, !_yellowLEDsState ); 
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Flash red LEDs.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::flashRedLEDs(                                )
{
  if (isInitialized(                   )) {
      digitalWrite(_redLEDsPin,    !_redLEDsState    );
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Flash blue LEDs.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::flashBlueLEDs(                                )
{
  if (isInitialized(                   )) {
      digitalWrite(_blueLEDsPin,   !_blueLEDsState  ); 
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn on the blue LEDs.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::turnOnBlueLEDs(     )
{
  if (isInitialized(                               )) {
     _blueLEDsState   = HIGH ;   // turn these LEDs on (HIGH is the voltage level)
      digitalWrite(_blueLEDsPin,   _blueLEDsState   );
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn off the blue LEDs.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::turnOffBlueLEDs(    )
{
  if (isInitialized(                               )) {
     _blueLEDsState   = LOW  ;   // turn these LEDs off (LOW is the voltage level)
      digitalWrite(_blueLEDsPin,   _blueLEDsState   );
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn on the green LEDs.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::turnOnGreenLEDs(     )
{
  if (isInitialized(                                )) {
     _greenLEDsState   = HIGH ;   // turn these LEDs on (HIGH is the voltage level)
      digitalWrite(_greenLEDsPin,   _greenLEDsState  );
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn off the green LEDs.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::turnOffGreenLEDs(    )
{
  if (isInitialized(                                )) {
     _greenLEDsState   = LOW  ;   // turn these LEDs off (LOW is the voltage level)
      digitalWrite(_greenLEDsPin,   _greenLEDsState  );
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn on the yellow LEDs.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::turnOnYellowLEDs(     )
{
  if (isInitialized(                                )) {
     _yellowLEDsState   = HIGH ;   // turn these LEDs on (HIGH is the voltage level)
      digitalWrite(_yellowLEDsPin,   _yellowLEDsState  );
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn off the yellow LEDs.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::turnOffYellowLEDs(    )
{
  if (isInitialized(                                )) {
     _yellowLEDsState   = LOW  ;   // turn these LEDs off (LOW is the voltage level)
      digitalWrite(_yellowLEDsPin,   _yellowLEDsState  );
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn on the red LEDs.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::turnOnRedLEDs(     )
{
  if (isInitialized(                                )) {
     _redLEDsState   = HIGH ;   // turn these LEDs on (HIGH is the voltage level)
      digitalWrite(_redLEDsPin,   _redLEDsState  );
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn off the red LEDs.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::turnOffRedLEDs(    )
{
  if (isInitialized(                                )) {
     _redLEDsState   = LOW  ;   // turn these LEDs off (LOW is the voltage level)
      digitalWrite(_redLEDsPin,   _redLEDsState  );
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

