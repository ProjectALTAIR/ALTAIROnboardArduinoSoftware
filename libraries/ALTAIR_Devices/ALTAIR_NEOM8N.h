/**************************************************************************/
/*!
    @file     ALTAIR_NEOM8N.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR NEO-M8N GPS receiver, located on the
    mast.

    Justin Albert  jalbert@uvic.ca     began on 6 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef   ALTAIR_NEOM8N_h
#define   ALTAIR_NEOM8N_h

#include "Arduino.h"
#include "ALTAIR_GPSSensor.h"
#include <TinyGPS++.h>

#define   NEOM8N_I2CADDRESS         0x42
#define   NEOM8N_MAXBUFFERSIZE        32
#define   NEOM8N_INITCODE           0xFD
#define   NEOM8N_INITBYTES             2
#define   NEOM8N_GETGPSCODE         0xFF
#define   NEOM8N_ERRORBYTE          0xFF

class ALTAIR_NEOM8N : public ALTAIR_GPSSensor {
  public:

    ALTAIR_NEOM8N(                    )    {                                          }

    virtual void      initialize(     )    {                                          }
    virtual bool      getGPS(         )                                             ;
    virtual uint8_t   typeAndHealth(  )    { return ((uint8_t) neom8n_healthy      ); }

    virtual double    lat(            )    { return           _gps.location.lat(   ); }
    virtual double    lon(            )    { return           _gps.location.lng(   ); }
    virtual long      ele(            )    { return           _gps.altitude.meters(); }  // In meters above mean sea level.
    virtual byte      hdop(           )    { return           _gps.hdop.value(     ); }  // Horizontal Degree Of Precision.  A number typically between 1 and 50.
    virtual uint32_t  age(            )    { return           _gps.location.age(   ); }
    virtual uint16_t  year(           )    { return           _gps.date.year(      ); }
    virtual uint8_t   month(          )    { return           _gps.date.month(     ); }
    virtual uint8_t   day(            )    { return           _gps.date.day(       ); }
    virtual uint8_t   hour(           )    { return           _gps.time.hour(      ); }
    virtual uint8_t   minute(         )    { return           _gps.time.minute(    ); }
    virtual uint8_t   second(         )    { return           _gps.time.second(    ); }

  private:

    TinyGPSPlus      _gps;

};
#endif    //   ifndef ALTAIR_NEOM8N_h
