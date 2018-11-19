/**************************************************************************/
/*!
    @file     ALTAIR_GPSSensor.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the base class for each of the two ALTAIR GPS receivers (the 
    NEO-M8N on the mast, and the DFRobot G6 located in a small plastic 
    housing directly atop the payload).

    Justin Albert  jalbert@uvic.ca     began on 6 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_GPSSensor_h
#define ALTAIR_GPSSensor_h

#include "Arduino.h"

typedef  enum { neom8n_healthy      = 0,
                dfrobotg6_healthy   = 1,
                neom8n_unhealthy    = 2,
                dfrobotg6_unhealthy = 3 } gpssensor_t;

class ALTAIR_GPSSensor {
  public:

    ALTAIR_GPSSensor(                              )     {  }

    virtual void            initialize(            ) = 0 ;
    virtual bool            getGPS(                ) = 0 ;
    virtual uint8_t         typeAndHealth(         ) = 0 ;

    virtual double          lat(                   ) = 0 ;
    virtual double          lon(                   ) = 0 ;
    virtual long            ele(                   ) = 0 ;    // In meters above mean sea level.
    virtual byte            hdop(                  ) = 0 ;    // Horizontal Degree Of Precision.  A number typically between 1 and 50.
    virtual uint32_t        age(                   ) = 0 ;
    virtual uint16_t        year(                  ) = 0 ;
    virtual uint8_t         month(                 ) = 0 ;
    virtual uint8_t         day(                   ) = 0 ;
    virtual uint8_t         hour(                  ) = 0 ;
    virtual uint8_t         minute(                ) = 0 ;
    virtual uint8_t         second(                ) = 0 ;
    virtual double          time(                  ) = 0 ;

  private:

};
#endif    //   ifndef ALTAIR_GPSSensor_h
