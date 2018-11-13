/**************************************************************************/
/*!
    @file     ALTAIR_OrientSensor.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL


    Justin Albert  jalbert@uvic.ca     began on 11 Nov. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#include "ALTAIR_OrientSensor.h"

/**************************************************************************/
/*!
 @brief  Function to convert int16_t output to int8_t output.  
*/
/**************************************************************************/
int8_t ALTAIR_OrientSensor::convertAccelInt16ToInt8( int16_t accel )
{
    int8_t     packedAccel;
    float      theAccelInSIUnits  =   accel / SHRTMAX_DIVBY_360;
    float      scaledAccel        =   theAccelInSIUnits * 3.;
    if (       theAccelInSIUnits <=   40.  &&  theAccelInSIUnits >= -40.) {
               packedAccel        =   scaledAccel;
    } else if (theAccelInSIUnits >    40.) {
               packedAccel        =  127;
    } else {
               packedAccel        = -128;
    }
    return     packedAccel;
}

/**************************************************************************/
/*!
 @brief  Function to convert int16_t output to uint8_t output.  
*/
/**************************************************************************/
uint8_t ALTAIR_OrientSensor::convertAccelInt16ToUInt8( int16_t accel )
{
    uint8_t    packedAccel;
    float      theAccelInSIUnits  =   accel / SHRTMAX_DIVBY_360;
    float      scaledAccel        =   theAccelInSIUnits * 3. + 120.;
    if (       theAccelInSIUnits <=   40.  &&  theAccelInSIUnits >= -40.) {
               packedAccel        =   scaledAccel;
    } else if (theAccelInSIUnits >    40.) {
               packedAccel        =  255;
    } else {
               packedAccel        =  254;
    }
    return     packedAccel;
}

/**************************************************************************/
/*!
 @brief  Function to convert int16_t output to uint8_t output.
*/
/**************************************************************************/
uint8_t ALTAIR_OrientSensor::convertYawInt16ToUInt8(  int16_t yaw   )
{
    uint8_t packedYaw;
    float   yawInDegrees = yaw / SHRTMAX_DIVBY_360;   // yawInDegrees should range from 0. to 360.
    float   scaledYaw    = yawInDegrees / 1.5;        // scaledYaw    should range from 0. to 240.
            packedYaw    = scaledYaw;
    return  packedYaw;
}

/**************************************************************************/
/*!
 @brief  Function to convert int16_t output to int8_t output.
*/
/**************************************************************************/
int8_t ALTAIR_OrientSensor::convertPitchRollInt16ToInt8( int16_t angle )
{
    int8_t packedAngle;
    float  angleInDegrees = angle / SHRTMAX_DIVBY_360;  // angleInDegrees should range from -90. to 90. (for pitch) or -180. to 180. (for roll)
    float  scaledAngle    = angleInDegrees * 2.;
    if (angleInDegrees <= 60. && angleInDegrees >= -60.) {
        packedAngle =  scaledAngle;
    } else if (angleInDegrees > 60.) {
        packedAngle =  127;
    } else {
        packedAngle = -128;
    }
    return packedAngle;
}

/**************************************************************************/
/*!
 @brief  Function to convert int16_t output to uint8_t output.
*/
/**************************************************************************/
uint8_t ALTAIR_OrientSensor::convertPitchRollInt16ToUInt8( int16_t angle )
{
    uint8_t packedAngle;
    float  angleInDegrees = angle / SHRTMAX_DIVBY_360;  // angleInDegrees should range from -90. to 90. (for pitch) or -180. to 180. (for roll)
    float  scaledAngle    = angleInDegrees * 2. + 120.;
    if (angleInDegrees <= 60. && angleInDegrees >= -60.) {
        packedAngle =  scaledAngle;
    } else if (angleInDegrees > 60.) {
        packedAngle =  255;
    } else {
        packedAngle =  254;
    }
    return packedAngle;
}
