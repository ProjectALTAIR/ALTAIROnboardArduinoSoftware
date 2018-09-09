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

#ifndef ALTAIR_NEOM8N_h
#define ALTAIR_NEOM8N_h

#include "Arduino.h"
#include "ALTAIR_GPSSensor.h"

class ALTAIR_NEOM8N : public ALTAIR_GPSSensor {
  public:

    ALTAIR_NEOM8N(                   )         {  }

    virtual void         initialize( )         {  }

  private:

};
#endif    //   ifndef ALTAIR_NEOM8N_h
