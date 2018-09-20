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

class TinyGPSPlus;

class ALTAIR_GPSSensor {
  public:

    ALTAIR_GPSSensor(                              )     {  }

    virtual void            initialize(            ) = 0 ;
    virtual bool            getGPS(TinyGPSPlus* gps) = 0 ;

  private:

};
#endif    //   ifndef ALTAIR_GPSSensor_h
