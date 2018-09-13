/**************************************************************************/
/*!
    @file     ALTAIR_SituatAwarenessSystem.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR "situational awareness" system, which
    includes the two GPS receivers (the NEO-M8N on the mast, and the 
    DFRobot G6 in a small plastic housing directly atop the payload); the 
    three orientation sensors (the Adafruit BNO055 accel/gyro/mag on the 
    mast, the CHRobotics UM7 accel/gyro/mag within the payload, and the
    SparkFun HMC6343 accel/mag on the mast); the three Adafruit BME280 
    pressure/temp/humidity sensors; and the many (primarily LM333) other
    temperature sensors located at various places within and around ALTAIR.

    This class is instantiated as a singleton via the instantiation of the
    (also singleton) ALTAIR_GlobalDeviceControl class.

    Justin Albert  jalbert@uvic.ca     began on 5 Sept. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_SituatAwarenessSystem_h
#define ALTAIR_SituatAwarenessSystem_h

#include "Arduino.h"
#include "ALTAIR_GPSSensors.h"
#include "ALTAIR_OrientSensors.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

class ALTAIR_SituatAwarenessSystem {
  public:

    ALTAIR_SituatAwarenessSystem(                        )                                    ;

    ALTAIR_GPSSensors*       gpsSensors(                 ) { return &_gpsSensors              ; }

    ALTAIR_OrientSensors*    orientSensors(              ) { return &_orientSensors           ; }

    Adafruit_BME280*         bmeMast(                    ) { return &_bmeMast                 ; }
    Adafruit_BME280*         bmeBalloon(                 ) { return &_bmeBalloon              ; }
    Adafruit_BME280*         bmePayload(                 ) { return &_bmePayload              ; }

    void                     initialize(                 )                                    ;
    void                     switchToOtherGPS(           ) { _gpsSensors.switchToOtherGPS()   ; }
    void                     switchToBackupOrientSensor1() { _orientSensors.switchToBackup1() ; }
    void                     switchToBackupOrientSensor2() { _orientSensors.switchToBackup2() ; }

  protected:

  private:
    ALTAIR_GPSSensors        _gpsSensors                                                      ;

    ALTAIR_OrientSensors     _orientSensors                                                   ;

    Adafruit_BME280          _bmeMast                                                         ;
    Adafruit_BME280          _bmeBalloon                                                      ;
    Adafruit_BME280          _bmePayload                                                      ;

};
#endif    //   ifndef ALTAIR_SituatAwarenessSystem_h
