/**************************************************************************/
/*!
    @file     ALTAIR_CutdownSystem.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR cutdown and parafoil steering system, 
    including the servo motor (a HobbyKing brand SW5513-6MA sail winch servo:
    https://hobbyking.com/en_us/sail-winch-servo-13kg-0-7sec-360deg-55g.html
    which has been modified to include an encoder to report its position 
    via an analog voltage).  It inherits from the base ALTAIR_ServoMotor
    class.

    This class is instantiated as a singleton via the instantiation of the
    (also singleton) ALTAIR_GlobalMotorControl class.

    Justin Albert  jalbert@uvic.ca     began on 2 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef   ALTAIR_CutdownSystem_h
#define   ALTAIR_CutdownSystem_h

#include "Arduino.h"
#include "ALTAIR_ServoMotor.h"

class ALTAIR_CutdownSystem : public ALTAIR_ServoMotor {
  public:

    ALTAIR_CutdownSystem(    byte   posADCPin  )                                 ;

    bool                     isCutdown(        )      { return _isCutdown        ; }

    void                     setCutdown(       )      {        _isCutdown = true ; }

    virtual float            maxSafeSetting(   )                                 ;
    virtual float            minSafeSetting(   )                                 ;

  protected:
    virtual void             resetPWMRegister( )                                 ;

  private:
    bool                     _isCutdown                                          ;

};
#endif    //   ifndef ALTAIR_CutdownSystem_h
