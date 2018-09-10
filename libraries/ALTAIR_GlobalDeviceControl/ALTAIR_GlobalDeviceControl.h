/**************************************************************************/
/*!
    @file     ALTAIR_GlobalDeviceControl.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This class contains methods for initializing and controlling all 
    ALTAIR devices which are not motors (nor part of the motor or servo 
    systems, e.g. the electronic speed controllers, etc), nor are light
    sources (nor are part of the light source systems, e.g. the
    photodiode light intensity monitors).  This includes: all 3 
    telemetry radios; the Adafruit MicroSD card data storage breakout 
    board; and all the position and orientation sensors (GPS, the
    accelerometer- and gyro-based yaw/pitch/roll sensors, and
    magnetometer), and environmental (temp, pressure, humidity) sensors.

    This class should be instantiated as a singleton.

    Justin Albert  jalbert@uvic.ca     began on 4 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_GlobalDeviceControl_h
#define ALTAIR_GlobalDeviceControl_h

#include "Arduino.h"

#include "ALTAIR_TelemetrySystem.h"
#include "ALTAIR_DataStorageSystem.h"
#include "ALTAIR_SituatAwarenessSystem.h"   // includes GPS, orientation, and environmental sensors

class ALTAIR_GlobalDeviceControl {
  public:

    ALTAIR_GlobalDeviceControl(                                             )                           ;

    bool                            initializeAllDevices(                   )                           ;

    void                            performCommand(       byte  commandByte )                           ;

    ALTAIR_TelemetrySystem&         telemSystem(                            ) { return     _telemSystem ; }
    ALTAIR_DataStorageSystem&       dataStoreSystem(                        ) { return _dataStoreSystem ; } // onboard SD card data storage
    ALTAIR_SituatAwarenessSystem&   sitAwareSystem(                         ) { return  _sitAwareSystem ; } // includes GPS, orientation, and environmental sensors

  protected:

  private:
    ALTAIR_TelemetrySystem         _telemSystem                                                         ;
    ALTAIR_DataStorageSystem       _dataStoreSystem                                                     ;
    ALTAIR_SituatAwarenessSystem   _sitAwareSystem                                                      ;

};
#endif    //   ifndef ALTAIR_GlobalDeviceControl_h
