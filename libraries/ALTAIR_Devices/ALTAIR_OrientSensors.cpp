/**************************************************************************/
/*!
    @file     ALTAIR_OrientSensors.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class containing all 4 of the ALTAIR orientation sensors:
    1) the Adafruit BNO055 (accel/gyro/mag), located on the mast; 2)
    the CHRobotics UM7 (accel/gyro/mag), located inside the payload; 3)
    the Sparkfun HMC6343 (accel/mag), located on the mast; and 4) the
    compass HMC5883L magnetometer (just mag), located inside the mast.

    Justin Albert  jalbert@uvic.ca     began on 6 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#include "ALTAIR_OrientSensors.h"

/**************************************************************************/
/*!
 @brief  Constructor.  Constructs the orientation sensor objects with
         their default arguments (via their respective default
         constructors).
*/
/**************************************************************************/
ALTAIR_OrientSensors::ALTAIR_OrientSensors(       )  
{
    _primary  = &_bno055                                ;
    _backup1  = &_um7                                   ;
    _backup2  = &_hmc6343                               ;
}

/**************************************************************************/
/*!
 @brief  Initialize the orientation sensors (within the setup routine).
*/
/**************************************************************************/
void ALTAIR_OrientSensors::initialize(            )
{
    _hmc5883l.initialize(                         )     ;
      _bno055.initialize(                         )     ;
         _um7.initialize(                         )     ;
     _hmc6343.initialize(                         )     ;
}

/**************************************************************************/
/*!
 @brief  Switch to backup orientation sensor #1.
*/
/**************************************************************************/
void ALTAIR_OrientSensors::switchToBackup1(       )
{
     ALTAIR_OrientSensor* formerPrimary = primary()     ;
                         _primary       = backup1()     ;
                         _backup1       = formerPrimary ;
}

/**************************************************************************/
/*!
 @brief  Switch to backup orientation sensor #2.
*/
/**************************************************************************/
void ALTAIR_OrientSensors::switchToBackup2(       )
{
     ALTAIR_OrientSensor* formerPrimary = primary()     ;
                         _primary       = backup2()     ;
                         _backup2       = backup1()     ;
                         _backup1       = formerPrimary ;
}

