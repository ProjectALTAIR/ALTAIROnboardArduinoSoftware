/**************************************************************************/
/*!
    @file     ALTAIR_HMC5883L.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR HMC5883L magnetometer orientation 
    sensor, located inside the mast.  This class derives from the 
    ALTAIR_OrientSensor generic orientation sensor base class.

    Justin Albert  jalbert@uvic.ca     began on 7 Sep. 2018

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_HMC5883L.h"
#include <Wire.h>

/**************************************************************************/
/*!
 @brief  (Default) constructor.
*/
/**************************************************************************/
ALTAIR_HMC5883L::ALTAIR_HMC5883L(                               )
{
}

/**************************************************************************/
/*!
 @brief  Initialize the orientation sensor during the setup routine.
*/
/**************************************************************************/
void ALTAIR_HMC5883L::initialize(                              ) 
{
    Serial.println(F("Starting I2C/TWI bus..."));
    Wire.begin();
    Serial.println(F("Initializing and calibrating I2C/TWI compass magnetometer..."));
/*
    const byte     compassmagCalibBytes[]    = { 0x71, 0x60, 0x01 };
    twiWriteBytes(compassmagI2CAddress, 0x00, compassmagCalibBytes, 1);
    delay(50);
    twiWriteBytes(compassmagI2CAddress, 0x01, &compassmagCalibBytes[1], 2);
    delay(100);
    getCompassmagData();
    const float    magScale[]                = { 1160.0, 1160.0, 1080 };
    magCalX = magScale[0]/abs(magDataX);
    magCalY = magScale[1]/abs(magDataY);
    magCalZ = magScale[2]/abs(magDataZ);
    twiWriteBytes(compassmagI2CAddress, 0x00, compassmagInitBytes, 3);
*/
    Wire.beginTransmission( HMC5883L_I2CADDRESS );
    Wire.write(             HMC5883L_INITCODE   );
    Wire.endTransmission(                       );
//    I2c.write(compassmagI2CAddress, 0x02, 0x00);
//    I2c.end();
}

/**************************************************************************/
/*!
 @brief  Return the heading, in degrees.
*/
/**************************************************************************/
float ALTAIR_HMC5883L::getHeading(                              )
{
    int16_t magDataX,      magDataY,      magDataZ;
    float   magCalX = 1.0, magCalY = 1.0, magCalZ = 1.0;                                                        

    Wire.beginTransmission( HMC5883L_I2CADDRESS );
    Wire.write(             HMC5883L_DATAREG    );
    Wire.endTransmission(                       );
    Wire.requestFrom(       HMC5883L_I2CADDRESS ,  HMC5883L_DATABYTES );
    magDataX  = Wire.read(                      ) << 8;
    magDataX |= Wire.read(                      );
    magDataZ  = Wire.read(                      ) << 8; // HMC5883L z data register is before the y register, don't ask y!...
    magDataZ |= Wire.read(                      );
    magDataY  = Wire.read(                      ) << 8;
    magDataY |= Wire.read(                      );

    return      atan2(magDataY*magCalY, magDataX*magCalX) * 180. / M_PI;
}
