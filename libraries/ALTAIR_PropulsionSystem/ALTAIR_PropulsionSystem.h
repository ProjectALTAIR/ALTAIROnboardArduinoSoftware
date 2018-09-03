/**************************************************************************/
/*!
    @file     ALTAIR_PropulsionSystem.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR propulsion system, including the
    four propulsion motors and their electronic speed controllers (ESCs),
    the rotation servo motor for the propulsion axle, the RPM and current 
    sensors for each propulsion motor, and the temperature sensors for 
    each propulsion motor and ESC.  This class contains each of those
    (sub-)device objects, and methods for accessing each of them.

    This class is instantiated as a singleton via the instantiation of the
    (also singleton) ALTAIR_GlobalMotorControl class.

    Justin Albert  jalbert@uvic.ca     began on 29 Aug. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_PropulsionSystem_h
#define ALTAIR_PropulsionSystem_h

#include "Arduino.h"
#include "ALTAIR_MotorAndESC.h"
#include "ALTAIR_PropAxleRotServo.h"

class ALTAIR_PropulsionSystem {
  public:

    ALTAIR_PropulsionSystem()                                                 ;

    bool                     isInitialized()                                  ;
    bool                     isRunning()                                      ;

    ALTAIR_MotorAndESC&      portOuterMotor()      { return _motorAndESC[0]   ; }
    ALTAIR_MotorAndESC&      portInnerMotor()      { return _motorAndESC[1]   ; }
    ALTAIR_MotorAndESC&      stbdInnerMotor()      { return _motorAndESC[2]   ; }
    ALTAIR_MotorAndESC&      stbdOuterMotor()      { return _motorAndESC[3]   ; }
    ALTAIR_MotorAndESC*      motors()              { return _motorAndESC      ; }

    ALTAIR_PropAxleRotServo& axleRotServo()        { return _propAxleRotServo ; }

    void                     initializePinModes()                             ;
    void                     initializePropControlRegisters()                 ;
    void                     initializePWMOutputRegisters()                   ;

  private:
    ALTAIR_MotorAndESC       _motorAndESC[4]                                  ;
    ALTAIR_PropAxleRotServo  _propAxleRotServo                                ;

};
#endif    //   ifndef ALTAIR_PropulsionSystem_h
