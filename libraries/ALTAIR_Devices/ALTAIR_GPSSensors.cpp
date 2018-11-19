/**************************************************************************/
/*!
    @file     ALTAIR_GPSSensors.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the two ALTAIR GPS receivers (the NEO-M8N on the 
    mast, and the DFRobot G6 located in a small plastic housing directly 
    atop the payload).

    Justin Albert  jalbert@uvic.ca     began on 6 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#include "ALTAIR_GPSSensors.h"
#include "ALTAIR_UM7.h"

/**************************************************************************/
/*!
 @brief  Constructor.  Constructs the two GPS receiver objects with
         their default arguments (via their respective default
         constructors).
*/
/**************************************************************************/
ALTAIR_GPSSensors::ALTAIR_GPSSensors(           )
{
    _primary  = &_neom8n                             ;
    _backup   = &_dfrobot                            ;
}

/**************************************************************************/
/*!
 @brief  Initialize both GPS receivers (within the setup routine).
*/
/**************************************************************************/
void ALTAIR_GPSSensors::initialize(             )
{
    _neom8n.initialize(                         )    ;
   _dfrobot.initialize(                         )    ;
}

/**************************************************************************/
/*!
 @brief  Switch primary and backup GPS.
*/
/**************************************************************************/
void ALTAIR_GPSSensors::switchToOtherGPS(       )
{
     ALTAIR_GPSSensor* formerPrimary = primary( )    ;
                      _primary       = backup(  )    ;
                      _backup        = formerPrimary ;
}
