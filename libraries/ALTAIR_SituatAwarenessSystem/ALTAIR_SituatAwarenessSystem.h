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
    SparkFun HMC6343 accel/mag on the mast); the Arduino Micro I2C slave
    board which reads and controls the 4 propulsion system RPM sensors, 
    4 propulsion system current sensors, and 8 propulsion system temp
    sensors; the three Adafruit BME280 pressure/temp/humidity sensors; and 
    the many (primarily LM333) other temperature sensors located at 
    various places within and around ALTAIR.

    This class is instantiated as a singleton via the instantiation of the
    (also singleton) ALTAIR_GlobalDeviceControl class.

    Justin Albert  jalbert@uvic.ca     began on 5 Sept. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef   ALTAIR_SituatAwarenessSystem_h
#define   ALTAIR_SituatAwarenessSystem_h

#include "Arduino.h"
#include "ALTAIR_GPSSensors.h"
#include "ALTAIR_OrientSensors.h"
#include "ALTAIR_ArduinoMicro.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define   SEALEVELPRESSURE_HPA       (1013.25)

class ALTAIR_SituatAwarenessSystem {
  public:

    ALTAIR_SituatAwarenessSystem(                        )                                    ;

    ALTAIR_GPSSensors*       gpsSensors(                 ) { return &_gpsSensors              ; }

    ALTAIR_OrientSensors*    orientSensors(              ) { return &_orientSensors           ; }

    ALTAIR_ArduinoMicro*     arduinoMicro(               ) { return &_arduinoMicro            ; }

    Adafruit_BME280*         bmeMast(                    ) { return &_bmeMast                 ; }
    Adafruit_BME280*         bmeBalloon(                 ) { return &_bmeBalloon              ; }
    Adafruit_BME280*         bmePayload(                 ) { return &_bmePayload              ; }

    void                     bmeMastPrintInfo(           ) { bme280PrintInfo( &_bmeMast     ) ; }
    void                     bmeBalloonPrintInfo(        ) { bme280PrintInfo( &_bmeBalloon  ) ; }
    void                     bmePayloadPrintInfo(        ) { bme280PrintInfo( &_bmePayload  ) ; }

    void                     initialize(                 )                                    ;
    void                     switchToOtherGPS(           ) { _gpsSensors.switchToOtherGPS(  ) ; }
    void                     switchToBackupOrientSensor1() { _orientSensors.switchToBackup1() ; }
    void                     switchToBackupOrientSensor2() { _orientSensors.switchToBackup2() ; }

  protected:

    void                     bme280PrintInfo(                Adafruit_BME280*   bme280      ) ;

  private:
    ALTAIR_GPSSensors        _gpsSensors                                                      ;

    ALTAIR_OrientSensors     _orientSensors                                                   ;

    ALTAIR_ArduinoMicro      _arduinoMicro                                                    ;

    Adafruit_BME280          _bmeMast                                                         ;
    Adafruit_BME280          _bmeBalloon                                                      ;
    Adafruit_BME280          _bmePayload                                                      ;

};
#endif    //   ifndef ALTAIR_SituatAwarenessSystem_h
