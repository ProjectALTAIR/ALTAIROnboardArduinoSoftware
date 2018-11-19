/**************************************************************************/
/*!
    @file     ALTAIR_DFRobotG6.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR DFRobot G6 GPS receiver, located in
    the small plastic housing directly atop the port side of the payload.

    Justin Albert  jalbert@uvic.ca     began on 6 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#include "ALTAIR_DFRobotG6.h"
#include "ALTAIR_UM7.h"


/**************************************************************************/
/*!
 @brief  Get the GPS, and place the data in the _lat, _lon, and _ele data
         members.  The DFRobot G6 is connected through the UM7, so we get
         the GPS from there.
*/
/**************************************************************************/
bool       ALTAIR_DFRobotG6::getGPS(                       )
{
    return ALTAIR_UM7::getGPS( &_lat, &_lon, &_ele, &_time );
}

/**************************************************************************/
/*!
 @brief  Convert the GPS time to the present UTC year.
*/
/**************************************************************************/
uint16_t   ALTAIR_DFRobotG6::year(                         )
{
    return 0;
}

/**************************************************************************/
/*!
 @brief  Convert the GPS time to the present UTC month.
*/
/**************************************************************************/
uint8_t   ALTAIR_DFRobotG6::month(                         )
{
    return 0;
}

/**************************************************************************/
/*!
 @brief  Convert the GPS time to the present UTC day.
*/
/**************************************************************************/
uint8_t   ALTAIR_DFRobotG6::day(                           )
{
    return 0;
}

/**************************************************************************/
/*!
 @brief  Convert the GPS time to the present UTC hour.
*/
/**************************************************************************/
uint8_t   ALTAIR_DFRobotG6::hour(                          )
{
    return ( (uint8_t)      (_time / 3600)    );
}

/**************************************************************************/
/*!
 @brief  Convert the GPS time to the present UTC minute.
*/
/**************************************************************************/
uint8_t   ALTAIR_DFRobotG6::minute(                        )
{
    return ( (uint8_t) fmod((_time / 60), 60) );
}

/**************************************************************************/
/*!
 @brief  Convert the GPS time to the present UTC second.
*/
/**************************************************************************/
uint8_t   ALTAIR_DFRobotG6::second(                        )
{
    return ( (uint8_t) fmod(_time, 60)        );
}

