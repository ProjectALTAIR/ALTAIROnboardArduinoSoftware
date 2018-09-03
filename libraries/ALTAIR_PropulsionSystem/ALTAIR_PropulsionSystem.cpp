/**************************************************************************/
/*!
    @file     ALTAIR_PropulsionSystem.cpp
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

    Justin Albert  jalbert@uvic.ca     began on 31 Aug. 2018

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_PropulsionSystem.h"
#include "ALTAIR_MotorPWMSettings.h"

/**************************************************************************/
/*!
 @brief  Constructor.  
*/
/**************************************************************************/
ALTAIR_PropulsionSystem::ALTAIR_PropulsionSystem()
{
    (_motorAndESC[0]).makePortOuter();
    (_motorAndESC[1]).makePortInner();
    (_motorAndESC[2]).makeStbdInner();
    (_motorAndESC[3]).makeStbdOuter();
}

/**************************************************************************/
/*!
 @brief  Initialize the propulsion system control pin modes after 
         power-on (within the setup routine).
*/
/**************************************************************************/
void ALTAIR_PropulsionSystem::initializePinModes(                                   )
{
    _propAxleRotServo.initializePinMode();
    for (int i = 0; i < 4; ++i) (_motorAndESC[i]).initializePinMode();
}

/**************************************************************************/
/*!
 @brief  Initialize the propulsion PWM control registers after power-on
         and after the control pin modes are initialized (within the 
         setup routine).  Note that this _only_ initializes the control
         registers for the 4 propulsion motors; the control registers for 
         the axle rotation servo, together with those for the other two 
         servo motors, are initialized directly within the 
         ALTAIR_MotorControl class.
*/
/**************************************************************************/
void ALTAIR_PropulsionSystem::initializePropControlRegisters(                      )
{
    PORT_MOTOR_PWMTIMER_REG_A = _BV(COM5A1) | _BV(COM5B1) | _BV(WGM52) | _BV(WGM51);
    PORT_MOTOR_PWMTIMER_REG_B = _BV(CS52);                                    
    STBD_MOTOR_PWMTIMER_REG_A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM12) | _BV(WGM11);
    STBD_MOTOR_PWMTIMER_REG_B = _BV(CS12);
}

/**************************************************************************/
/*!
 @brief  Initialize the propulsion system PWM output registers after
         power-on, after the control pin modes are initialized, and
         after the PWM control registers are initialized (within the 
         setup routine).
*/
/**************************************************************************/
void ALTAIR_PropulsionSystem::initializePWMOutputRegisters(                        )
{
    _propAxleRotServo.initializePWMRegister();
    for (int i = 0; i < 4; ++i) (_motorAndESC[i]).initializePWMRegister();
}

/**************************************************************************/
/*!
 @brief  Returns true if _any_ of the propulsion motors is spinning.  (Note:
         does NOT return true [but rather false] if the axle rotation servo
         is rotating, however none of the propulsion motors is spinning.)
*/
/**************************************************************************/
bool ALTAIR_PropulsionSystem::isRunning(                                          )
{
    return ( portOuterMotor().isRunning()     || 
             portInnerMotor().isRunning()     ||
             stbdInnerMotor().isRunning()     ||
             stbdOuterMotor().isRunning()       );
}

/**************************************************************************/
/*!
 @brief  Returns true _only_ if _all_ of the motors are initialized.  
*/
/**************************************************************************/
bool ALTAIR_PropulsionSystem::isInitialized(                                      )
{
    return (   axleRotServo().isInitialized() &&
             portOuterMotor().isInitialized() && 
             portInnerMotor().isInitialized() &&
             stbdInnerMotor().isInitialized() &&
             stbdOuterMotor().isInitialized()   );
}
