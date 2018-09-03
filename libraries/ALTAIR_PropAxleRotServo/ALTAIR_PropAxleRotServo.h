/**************************************************************************/
/*!
    @file     ALTAIR_PropAxleRotServo.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR propulsion axle rotation servo motor
    (a Corona CS238MG -- actually, its older product code is CS-238HV, 
    but I believe it is the same device:
    https://hobbyking.com/en_us/corona-cs238mg-metal-gear-servo-4-6kg-0-14sec-22g.html
    which has been modified to include an encoder to report its position 
    via an analog voltage).  It inherits from the base ALTAIR_ServoMotor
    class.

    This class will be instantiated (as a singleton) by the instantiation
    of the ALTAIR_PropulsionSystem singleton class. 

    Justin Albert  jalbert@uvic.ca     began on 2 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef   ALTAIR_PropAxleRotServo_h
#define   ALTAIR_PropAxleRotServo_h

#include "Arduino.h"
#include "ALTAIR_ServoMotor.h"


class ALTAIR_PropAxleRotServo : public ALTAIR_ServoMotor {
  public:

    ALTAIR_PropAxleRotServo(                )                          ;

  protected:
    virtual void     resetPWMRegister(      )                          ;


  private:

};
#endif    //   ifndef ALTAIR_PropAxleRotServo_h
