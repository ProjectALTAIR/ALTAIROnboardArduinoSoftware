/**************************************************************************/
/*!
    @file     ALTAIR_HMC5883L.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR HMC5883L magnetometer orientation 
    sensor, located inside the mast.  This class derives from the 
    ALTAIR_OrientSensor generic orientation sensor base class.

    Justin Albert  jalbert@uvic.ca     began on 7 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/
#ifndef   ALTAIR_HMC5883L_h
#define   ALTAIR_HMC5883L_h

#include "ALTAIR_OrientSensor.h"
#include <Adafruit_HMC5883_U.h>

#define   HMC5883L_SENSORID    12345

class ALTAIR_HMC5883L : public ALTAIR_OrientSensor {
  public:

    ALTAIR_HMC5883L(                                        );

    virtual void               initialize(                  );
            void               update(                      ) { _theHMC5883.getEvent(     &_lastEvent               ); }

            float              getHeading(                  );

            int16_t            accelZ(                      ) { return convertFloatToInt16(_lastEvent.acceleration.z); }
            int16_t            accelX(                      ) { return convertFloatToInt16(_lastEvent.acceleration.x); }
            int16_t            accelY(                      ) { return convertFloatToInt16(_lastEvent.acceleration.y); }
            int16_t            yaw(                         ) { return convertFloatToInt16(_lastEvent.orientation.z ); }
            int16_t            pitch(                       ) { return convertFloatToInt16(_lastEvent.orientation.y ); }
            int16_t            roll(                        ) { return convertFloatToInt16(_lastEvent.orientation.x ); }
            int8_t             temperature(                 ) { return convertFloatToInt16(_lastEvent.temperature   ); }
            // Must assume it is healthy, since there is no error status reporting on the HMC5883L.
            uint8_t            typeAndHealth(               ) { return ((uint8_t)           hmc5883l_healthy        ); } 

  protected:

  private:
    Adafruit_HMC5883_Unified  _theHMC5883 ;
    sensors_event_t           _lastEvent  ;
};
#endif
