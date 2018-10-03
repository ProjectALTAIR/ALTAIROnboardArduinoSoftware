/**************************************************************************/
/*!
    @file     ALTAIR_BleedSystem.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR balloon helium bleed valve and 
    internal balloon environment monitoring system, including the bleed
    valve servo motor (a Corona DS538HV
    https://hobbyking.com/en_us/corona-ds538hv-digital-metal-gear-servo-8kg-0-12sec-58g.html
    which [like the other servos] has been modified to include an encoder 
    to report its position via an analog voltage), the Adafruit BME280 
    pressure, temp, and humidity sensor located within the balloon 
    envelope inside the valve, and the LM333 temp sensor also located 
    within the balloon envelope inside the valve.  It inherits from the 
    base ALTAIR_ServoMotor class.

    This class is instantiated as a singleton via the instantiation of the
    (also singleton) ALTAIR_GlobalMotorControl class. 

    Justin Albert  jalbert@uvic.ca     began on 2 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef   ALTAIR_BleedSystem_h
#define   ALTAIR_BleedSystem_h

#include "Arduino.h"
#include "ALTAIR_ServoMotor.h"

class ALTAIR_BleedSystem : public ALTAIR_ServoMotor {
  public:

    ALTAIR_BleedSystem(           byte  posADCPin )                    ;

    bool                 isOpen(                  )   { return _isOpen ; }

//    void                 openBleedValve(           )                    ;    will implement later!
//    void                 closeBleedValve(          )                    ;

    virtual float        maxSafeSetting(          )                    ;
    virtual float        minSafeSetting(          )                    ;

  protected:
    virtual void         resetPWMRegister(        )                    ;


  private:
    bool                _isOpen                                        ;

};
#endif    //   ifndef ALTAIR_BleedSystem_h
