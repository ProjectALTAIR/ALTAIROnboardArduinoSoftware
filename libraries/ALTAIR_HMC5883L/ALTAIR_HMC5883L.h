/**************************************************************************/
/*!
    @file     ALTAIR_HMC5883L.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR HMC5883L magnetometer orientation 
    sensor, located inside the mast.  This class derives from the 
    ALTAIR_OrientSensor generic orientation sensor base class.

    Justin Albert  jalbert@uvic.ca     began on 7 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/
#ifndef   ALTAIR_HMC5883L_h
#define   ALTAIR_HMC5883L_h

#include "ALTAIR_OrientSensor.h"

#define   HMC5883L_I2CADDRESS  0x1E
#define   HMC5883L_INITCODE    0x02

class ALTAIR_HMC5883L : public ALTAIR_OrientSensor {
  public:

    ALTAIR_HMC5883L(                                        );

    virtual void      initialize(                           );

  protected:

  private:
};
#endif
