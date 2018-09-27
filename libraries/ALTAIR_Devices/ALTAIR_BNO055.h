/**************************************************************************/
/*!
    @file     ALTAIR_BNO055.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR Adafruit BNO055 orientation sensor, 
    located on the mast.  This class derives from the ALTAIR_OrientSensor 
    generic orientation sensor base class.

    Justin Albert  jalbert@uvic.ca     began on 6 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/
#ifndef   ALTAIR_BNO055_h
#define   ALTAIR_BNO055_h

#include "ALTAIR_OrientSensor.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define   DEFAULT_BNO055_ADAFRUITID   55

class ALTAIR_BNO055 : public ALTAIR_OrientSensor {
  public:

    ALTAIR_BNO055(                byte    adafruitSensorID  )                                                        ;
    ALTAIR_BNO055(                                          )                                                        ; // No argument => default value.

    Adafruit_BNO055*  theBNO055(                            ) { return                    &_theBNO055                ; }

    virtual void      initialize(                           )                                                        ;
            void      update(                               ) { _theBNO055.getEvent(      &_lastEvent               ); }
            void      printInfo(                            )                                                        ;

    int16_t            accelZ(                              ) { return convertFloatToInt16(_lastEvent.acceleration.z); }
    int16_t            accelX(                              ) { return convertFloatToInt16(_lastEvent.acceleration.x); }
    int16_t            accelY(                              ) { return convertFloatToInt16(_lastEvent.acceleration.y); }
    int16_t            yaw(                                 ) { return convertFloatToInt16(_lastEvent.orientation.z ); }
    int16_t            pitch(                               ) { return convertFloatToInt16(_lastEvent.orientation.y ); }
    int16_t            roll(                                ) { return convertFloatToInt16(_lastEvent.orientation.x ); }
    int8_t             temperature(                         ) { return convertFloatToInt16(_lastEvent.temperature   ); }
    uint8_t            typeAndHealth(                       )                                                        ;

  protected:

  private:
    // this class is basically just a container for the Adafruit_BNO055 class
    Adafruit_BNO055  _theBNO055                                                    ;
    sensors_event_t  _lastEvent                                                    ;
};
#endif
