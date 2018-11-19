/**************************************************************************/
/*!
    @file     ALTAIR_DFRobotG6.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR DFRobot G6 GPS receiver, located in
    the small plastic housing directly atop the port side of the payload.

    Justin Albert  jalbert@uvic.ca     began on 6 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_DFRobotG6_h
#define ALTAIR_DFRobotG6_h

#include "Arduino.h"
#include "ALTAIR_GPSSensor.h"

class ALTAIR_DFRobotG6 : public ALTAIR_GPSSensor {
  public:

    ALTAIR_DFRobotG6(                 )  {  }

    virtual void      initialize(     )  {  }
    virtual bool      getGPS(         )  ;
    virtual uint8_t   typeAndHealth(  )  { return ((uint8_t) dfrobotg6_healthy); }

    virtual double    lat(            )  { return           _lat               ; }
    virtual double    lon(            )  { return           _lon               ; }
    virtual long      ele(            )  { return           _ele               ; }  // In meters above mean sea level.
    virtual byte      hdop(           )  { return            0                 ; }  // Horizontal Degree Of Precision.  A number typically between 1 and 50.
    virtual uint32_t  age(            )  { return            0                 ; }
    virtual uint16_t  year(           )  ;
    virtual uint8_t   month(          )  ;
    virtual uint8_t   day(            )  ;
    virtual uint8_t   hour(           )  ;
    virtual uint8_t   minute(         )  ;
    virtual uint8_t   second(         )  ;

  private:

    double           _lat                ;
    double           _lon                ;
    double           _ele                ;
    double           _time               ;  // time in seconds since 0000 UT on January 6, 1980

};
#endif    //   ifndef ALTAIR_DFRobotG6_h
