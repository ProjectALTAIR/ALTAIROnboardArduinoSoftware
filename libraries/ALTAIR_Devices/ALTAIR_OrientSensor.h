/**************************************************************************/
/*!
    @file     ALTAIR_OrientSensor.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the base class for each of the ALTAIR orientation sensors:

    Justin Albert  jalbert@uvic.ca     began on 6 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_OrientSensor_h
#define ALTAIR_OrientSensor_h

#include "Arduino.h"

class ALTAIR_OrientSensor {
  public:

    ALTAIR_OrientSensor(                 )     {  }

    virtual void            initialize(  ) = 0    ;

  private:

};
#endif    //   ifndef ALTAIR_OrientSensor_h
