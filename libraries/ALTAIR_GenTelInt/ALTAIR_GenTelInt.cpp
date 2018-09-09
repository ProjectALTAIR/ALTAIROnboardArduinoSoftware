/**************************************************************************/
/*!
    @file     ALTAIR_GenTelInt.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the Generic Telemetry Interface base class for ALTAIR.
    Justin Albert  jalbert@uvic.ca     began on 8 Oct. 2017

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_GenTelInt.h"

#include <TinyGPS++.h>

/**************************************************************************/
/*!
 @brief  Constructor.  This will always be overridden (with the necessary 
         transceiver-specific code) in the derived classes.
*/
/**************************************************************************/
ALTAIR_GenTelInt::ALTAIR_GenTelInt() 
{
}

/**************************************************************************/
/*!
 @brief  Send GPS info.
*/
/**************************************************************************/
bool ALTAIR_GenTelInt::sendGPS(TinyGPSPlus& gps) 
{
    int32_t latitude  = gps.location.lat() * 1000000;  // Latitude,  in millionths of a degree.
    int32_t longitude = gps.location.lng() * 1000000;  // Longitude, in millionths of a degree.
    int16_t elevation = gps.altitude.meters();         // Elevation above mean sea level in meters.  NOTE: as this is signed 16 bit, this will *turn over*
                                                       // when above 32.77 km!  Would need to add & transmit an extra byte, if one preferred a higher limit.
                                                       // (Or make it unsigned, which would cause issues for launches from Death Valley or the Dead Sea. :)
                                                       // (One could get clever, and make it unsigned, but add, then later remove, 1000 meters, but I'm not 
                                                       //  that clever.)
    sendStart();
    send(0x0B);

  //send latitude
    send(byte(( latitude >> 24) & 0xFF));
    send(byte(( latitude >> 16) & 0xFF));
    send(byte(( latitude >>  8) & 0xFF));
    send(byte(  latitude        & 0xFF));
  //send longitude
    send(byte((longitude >> 24) & 0xFF));
    send(byte((longitude >> 16) & 0xFF));
    send(byte((longitude >>  8) & 0xFF));
    send(byte( longitude        & 0xFF));
  //send elevation
    send(byte((elevation >>  8) & 0xFF));
    send(byte( elevation        & 0xFF));

    return send('T');
}
