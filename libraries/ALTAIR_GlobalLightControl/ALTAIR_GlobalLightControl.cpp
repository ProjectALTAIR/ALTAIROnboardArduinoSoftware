/**************************************************************************/
/*!
    @file     ALTAIR_GlobalLightControl.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL


    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_GlobalLightControl.h"

/**************************************************************************/
/*!
 @brief  Constructor. 
*/
/**************************************************************************/
ALTAIR_GlobalLightControl::ALTAIR_GlobalLightControl(                                        )
{
}

/**************************************************************************/
/*!
 @brief  Fully initialize all ALTAIR devices within these systems.
*/
/**************************************************************************/
bool ALTAIR_GlobalLightControl::initializeAllLightSources(                                   )
{
    _intSphereSource.initialize()  ;
      _diffLEDSource.initialize()  ;
     _lightSourceMon.initialize()  ;
       return        true          ;
}

/**************************************************************************/
/*!
 @brief  Perform a command.
*/
/**************************************************************************/
void ALTAIR_GlobalLightControl::performCommand(       byte                  commandByte      )
{
  switch(commandByte) {
    case 'A':
      _intSphereSource.turnOnBlueLaser();
       break;
    case 'a':
      _intSphereSource.turnOffBlueLaser();
       break;
    case 'B':
      _intSphereSource.turnOnGreenLaser();
       break;
    case 'b':
      _intSphereSource.turnOffGreenLaser();
       break;
    case 'C':
      _intSphereSource.turnOnRed635nmLaser();
       break;
    case 'c':
      _intSphereSource.turnOffRed635nmLaser();
       break;
    case 'D':
      _intSphereSource.turnOnRed670nmLaser();
       break;
    case 'd':
      _intSphereSource.turnOffRed670nmLaser();
       break;
    case 'F':
      _diffLEDSource.turnOnBlueLEDs();
       break;
    case 'f':
      _diffLEDSource.turnOffBlueLEDs();
       break;
    case 'G':
      _diffLEDSource.turnOnGreenLEDs();
       break;
    case 'g':
      _diffLEDSource.turnOffGreenLEDs();
       break;
    case 'H':
      _diffLEDSource.turnOnYellowLEDs();
       break;
    case 'h':
      _diffLEDSource.turnOffYellowLEDs();
       break;
    case 'I':
      _diffLEDSource.turnOnRedLEDs();
       break;
    case 'i':
      _diffLEDSource.turnOffRedLEDs();
       break;
// Add light source monitoring commands here...
    default :
       break;
  }
}


