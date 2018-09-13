/**************************************************************************/
/*!
    @file     ALTAIR_HMC6343.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR Sparkfun HMC6343 orientation sensor, 
    located on the mast.  This class derives from the ALTAIR_OrientSensor 
    generic orientation sensor base class.

    Justin Albert  jalbert@uvic.ca     began on 7 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/
#ifndef   ALTAIR_HMC6343_h
#define   ALTAIR_HMC6343_h

#include "ALTAIR_OrientSensor.h"
#include <SFE_HMC6343.h>

class ALTAIR_HMC6343 : public ALTAIR_OrientSensor {
  public:

    ALTAIR_HMC6343(                   )                      ;

    virtual void      initialize(     )                      ;

    SFE_HMC6343&      theHMC6343(     ) { return _theHMC6343 ; }

  protected:

  private:
    // this class is basically just a container for the Sparkfun SFE_HMC6343 class
    SFE_HMC6343      _theHMC6343                             ;
};
#endif
