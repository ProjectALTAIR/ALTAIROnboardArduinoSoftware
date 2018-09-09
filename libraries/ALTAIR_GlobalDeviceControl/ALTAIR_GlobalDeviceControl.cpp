/**************************************************************************/
/*!
    @file     ALTAIR_GlobalDeviceControl.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This class contains methods for initializing and controlling all 
    ALTAIR devices which are not motors (nor part of the motor or servo 
    systems, e.g. the electronic speed controllers, etc), nor are light
    sources (nor are part of the light source systems, e.g. the
    photodiode light intensity monitors).  This includes: all 3 
    telemetry radios, and all the position and orientation sensors (GPS,
    the accelerometer- and gyro-based yaw/pitch/roll sensors, and
    magnetometer), and environmental (temp, pressure, humidity) sensors.

    This class should be instantiated as a singleton.

    Justin Albert  jalbert@uvic.ca     began on 4 Sep. 2018

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_GlobalDeviceControl.h"

/**************************************************************************/
/*!
 @brief  Constructor. 
*/
/**************************************************************************/
ALTAIR_GlobalDeviceControl::ALTAIR_GlobalDeviceControl(                                      )
{
}

/**************************************************************************/
/*!
 @brief  Fully initialize all ALTAIR devices within these systems.
*/
/**************************************************************************/
bool ALTAIR_GlobalDeviceControl::initializeAllDevices(                                       )
{
       _telemSystem.initialize()                   ;
    _sitAwareSystem.initialize()                   ;

        return      true                           ;
}

/**************************************************************************/
/*!
 @brief  Perform a command.
*/
/**************************************************************************/
void ALTAIR_GlobalDeviceControl::performCommand(       byte                  commandByte      )
{
  switch(commandByte) {
    case 'G':
    case 'g':
      _sitAwareSystem.switchToOtherGPS();
       break;
    case 'O':
      _sitAwareSystem.switchToBackupOrientSensor1();
       break;
    case 'o':
      _sitAwareSystem.switchToBackupOrientSensor2();
       break;
    case 'R':
      _telemSystem.switchToBackup1();
       break;
    case 'r':
      _telemSystem.switchToBackup2();
       break;
    default :
       break;
  }
}


