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
  _greenLEDsPin(                                                greenLEDsPin  )
{

}

/**************************************************************************/
/*!
 @brief  Default constructor (unused).  
*/
/**************************************************************************/
ALTAIR_DiffLEDLightSource::ALTAIR_DiffLEDLightSource(                         ) :
  _yellowLEDsPin(                                    DEFAULT_YELLOWLEDSPIN    ) ,
  _redLEDsPin(                                       DEFAULT_REDLEDSPIN       ) ,
  _blueLEDsPin(                                      DEFAULT_BLUELEDSPIN      ) ,
  _greenLEDsPin(                                     DEFAULT_GREENLEDSPIN     )
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

// normal situation: flash yellow LEDs then NO lights on (formerly it was yellow LEDs and green  
// laser on, but that heats up the I-drive transistor too much)
  setLightsNormal(              );
}


/**************************************************************************/
/*!
 @brief  Set the light source output to its normal state.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::setLightsNormal(                              )
{
// Normal situation: flash yellow LEDs then NO lights on (formerly it was yellow LEDs and green  
// laser on, but that heats up the I-drive transistor too much).
  flashYellowThenTurnOffLEDs();
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
  turnOnRedLEDs();
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
  turnOnBlueLEDs();
}

/**************************************************************************/
/*!
 @brief  Turn off all the LED light sources.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::turnOffLEDs(                                  )
{
  if (isInitialized(                  )) {
      digitalWrite(_yellowLEDsPin, LOW);   // turn these LEDs off (LOW is the voltage level)
      digitalWrite(_redLEDsPin,    LOW);   // turn these LEDs off (LOW is the voltage level)
      digitalWrite(_blueLEDsPin,   LOW);   // turn these LEDs off (LOW is the voltage level)
      digitalWrite(_greenLEDsPin,  LOW);   // turn these LEDs off (LOW is the voltage level)   
      turnOff(                        );
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Flash yellow (for 20 ms) then turn off all the LED light sources.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::flashYellowThenTurnOffLEDs(                   )
{
  if (isInitialized(                   )) {
      turnOn(                          );
      digitalWrite(_yellowLEDsPin, HIGH);  // turn these LEDs on (HIGH is the voltage level)
      digitalWrite(_redLEDsPin,    LOW );  // turn these LEDs off (LOW is the voltage level)
      digitalWrite(_blueLEDsPin,   LOW );  // turn these LEDs off (LOW is the voltage level)
      digitalWrite(_greenLEDsPin,  LOW );  // turn these LEDs off (LOW is the voltage level)
      delay(20);
      digitalWrite(_yellowLEDsPin, LOW );  // turn these LEDs off (LOW is the voltage level)
      turnOff(                         );
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn on the red (and only the red) LEDs.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::turnOnRedLEDs(                                )
{
  if (isInitialized(                   )) {
      turnOn(                          );
      digitalWrite(_yellowLEDsPin, LOW );  // turn these LEDs off (LOW is the voltage level)
      digitalWrite(_redLEDsPin,    HIGH);  // turn these LEDs on (HIGH is the voltage level)
      digitalWrite(_blueLEDsPin,   LOW );  // turn these LEDs off (LOW is the voltage level)
      digitalWrite(_greenLEDsPin,  LOW );  // turn these LEDs off (LOW is the voltage level)
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

/**************************************************************************/
/*!
 @brief  Turn on the blue (and only the blue) LEDs.
*/
/**************************************************************************/
void ALTAIR_DiffLEDLightSource::turnOnBlueLEDs(                                )
{
  if (isInitialized(                   )) {
      turnOn(                          );
      digitalWrite(_yellowLEDsPin, LOW );  // turn these LEDs off (LOW is the voltage level)
      digitalWrite(_redLEDsPin,    LOW );  // turn these LEDs off (LOW is the voltage level)
      digitalWrite(_blueLEDsPin,   HIGH);  // turn these LEDs on (HIGH is the voltage level)
      digitalWrite(_greenLEDsPin,  LOW );  // turn these LEDs off (LOW is the voltage level)
  }
// There probably should be some sort of error if the light source is not initialized yet...
}

