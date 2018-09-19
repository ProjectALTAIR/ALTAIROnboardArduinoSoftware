/**************************************************************************/
/*!
    @file     ALTAIR_HMC6343.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR Sparkfun HMC6343 orientation sensor,
    located on the mast.  This class derives from the ALTAIR_OrientSensor
    generic orientation sensor base class.

    Justin Albert  jalbert@uvic.ca     began on 7 Sep. 2018

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_HMC6343.h"

/**************************************************************************/
/*!
 @brief  (Default) constructor.
*/
/**************************************************************************/
ALTAIR_HMC6343::ALTAIR_HMC6343(                               )
{
}

/**************************************************************************/
/*!
 @brief  Initialize the orientation sensor during the setup routine.
*/
/**************************************************************************/
void ALTAIR_HMC6343::initialize(                              ) 
{
    Serial.println(F("HMC6343 magnetometer initialization..."));

    if (!_theHMC6343.init()) {
        Serial.println("SparkFun HMC6343 magnetometer initialization failed\n\r"); // Report failure, is the sensor wiring correct?
        while(1);
    } 
}

/**************************************************************************/
/*!
 @brief  Read and print out both the heading and the acceleration data.
*/
/**************************************************************************/
void ALTAIR_HMC6343::printInfo(                              )
{
    _theHMC6343.readHeading( );
    printHeadingData(        );
    _theHMC6343.readAccel(   );
    printAccelData(          );
}

/**************************************************************************/
/*!
 @brief  Print out the heading data.
*/
/**************************************************************************/
void ALTAIR_HMC6343::printHeadingData(                       )
{
    Serial.print(F("Heading: "));
    Serial.print(_theHMC6343.heading); Serial.print(F("  ")); // Print raw heading value
    Serial.print((float) _theHMC6343.heading/10.0);Serial.print(F(" degrees"));Serial.println(); // Print heading in degrees
}

/**************************************************************************/
/*!
 @brief  Print out the acceleration data.
*/
/**************************************************************************/
void ALTAIR_HMC6343::printAccelData(                         )
{
    Serial.print(F("HMC6343 X: "));
    Serial.print(_theHMC6343.accelX); Serial.print(F("  ")); // Print raw acceleration measurement on x axis
    Serial.print((float) _theHMC6343.accelX/1024.0);Serial.println(F("g")); // Print x axis acceleration measurement in g forces
    Serial.print(F("HMC6343 Y: "));
    Serial.print(_theHMC6343.accelY); Serial.print(F("  ")); // Print raw acceleration measurement on y axis
    Serial.print((float) _theHMC6343.accelY/1024.0);Serial.println(F("g")); // Print y axis acceleration measurement in g forces
    Serial.print(F("HMC6343 Z: "));
    Serial.print(_theHMC6343.accelZ); Serial.print(F("  ")); // Print raw acceleration measurement on z axis
    Serial.print((float) _theHMC6343.accelZ/1024.0);Serial.println(F("g")); // Print z axis acceleration measurement in g forces
}
