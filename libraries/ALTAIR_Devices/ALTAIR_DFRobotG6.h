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

    ALTAIR_DFRobotG6(                             )  {  }

    virtual void         initialize(              )  {  }
    virtual bool         getGPS( TinyGPSPlus* gps )  ;


  private:

};
#endif    //   ifndef ALTAIR_DFRobotG6_h
