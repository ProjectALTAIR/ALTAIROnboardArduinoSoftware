/**************************************************************************/
/*!
    @file     ALTAIR_OrientSensor.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the base class for each of the ALTAIR orientation sensors:
    1) the Adafruit BNO055 (accel/gyro/mag), located on the mast; 2) 
    the CHRobotics UM7 (accel/gyro/mag), located inside the payload; 3)
    the Sparkfun HMC6343 (accel/mag), located on the mast; and 4) the
    compass HMC5883L magnetometer (just mag), located inside the mast.

    Justin Albert  jalbert@uvic.ca     began on 6 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef  ALTAIR_OrientSensor_h
#define  ALTAIR_OrientSensor_h

#include "Arduino.h"
#include <Adafruit_Sensor.h>

#define  SHRTMAX_DIVBY_360          91.02222                 // = 2^15 / 360.

typedef  enum { bno055_healthy     = 0,
                um7_healthy        = 1,
                hmc6343_healthy    = 2,
                hmc5883l_healthy   = 3,
                bno055_unhealthy   = 4,
                um7_unhealthy      = 5,
                hmc6343_unhealthy  = 6,
                hmc5883l_unhealthy = 7 } orientsensor_t;

class ALTAIR_OrientSensor {
  public:

    ALTAIR_OrientSensor(                                               ) {                                                }

    virtual void           initialize(                                 ) = 0                                            ;
    virtual void           update(                                     ) = 0                                            ;
            int16_t        convertFloatToInt16(          float   data  ) { return (int16_t) ( data * SHRTMAX_DIVBY_360 ); }
            int8_t         convertAccelInt16ToInt8(      int16_t accel )                                                ;
            uint8_t        convertAccelInt16ToUInt8(     int16_t accel )                                                ;
            uint8_t        convertYawInt16ToUInt8(       int16_t yaw   )                                                ;
            int8_t         convertPitchRollInt16ToInt8(  int16_t angle )                                                ;
            uint8_t        convertPitchRollInt16ToUInt8( int16_t angle )                                                ;

    virtual int16_t        accelZ(                                     ) = 0                                            ;
    virtual int16_t        accelX(                                     ) = 0                                            ;
    virtual int16_t        accelY(                                     ) = 0                                            ;
    virtual int16_t        yaw(                                        ) = 0                                            ;
    virtual int16_t        pitch(                                      ) = 0                                            ;
    virtual int16_t        roll(                                       ) = 0                                            ;
    virtual int8_t         temperature(                                ) = 0                                            ;
    virtual uint8_t        typeAndHealth(                              ) = 0                                            ;

            int8_t         accelZInt8(                                 ) { return convertAccelInt16ToInt8(    accelZ( )); }
            int8_t         accelXInt8(                                 ) { return convertAccelInt16ToInt8(    accelX( )); }
            int8_t         accelYInt8(                                 ) { return convertAccelInt16ToInt8(    accelY( )); }
            uint8_t        accelZUInt8(                                ) { return convertAccelInt16ToUInt8(   accelZ( )); }
            uint8_t        accelXUInt8(                                ) { return convertAccelInt16ToUInt8(   accelX( )); }
            uint8_t        accelYUInt8(                                ) { return convertAccelInt16ToUInt8(   accelY( )); }
            uint8_t        yawUInt8(                                   ) { return convertYawInt16ToUInt8(      yaw(   )); }
            int8_t         pitchInt8(                                  ) { return convertPitchRollInt16ToInt8( pitch( )); }
            int8_t         rollInt8(                                   ) { return convertPitchRollInt16ToInt8( roll(  )); }
            uint8_t        pitchUInt8(                                 ) { return convertPitchRollInt16ToUInt8(pitch( )); }
            uint8_t        rollUInt8(                                  ) { return convertPitchRollInt16ToUInt8(roll(  )); }

  private:

};
#endif    //   ifndef ALTAIR_OrientSensor_h
