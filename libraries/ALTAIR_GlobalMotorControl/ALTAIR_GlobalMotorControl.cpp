/**************************************************************************/
/*!
    @file     ALTAIR_GlobalMotorControl.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This class contains methods for initializing and controlling 
    _all_ ALTAIR motors (including both the 4 propulsion motors as well 
    as the 3 servo motors) all at once.

    This class should be instantiated as a singleton.

    Justin Albert  jalbert@uvic.ca     began on 2 Sep. 2018

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_GlobalMotorControl.h"
#include "ALTAIR_MotorPWMSettings.h"

/**************************************************************************/
/*!
 @brief  Constructor. 
*/
/**************************************************************************/
ALTAIR_GlobalMotorControl::ALTAIR_GlobalMotorControl(                                        )
{
}

/**************************************************************************/
/*!
 @brief  Fully initialize all ALTAIR motors.
*/
/**************************************************************************/
void ALTAIR_GlobalMotorControl::initializeAllMotors(                                         )
{
      _bleedSystem.initializePinMode();
    _cutdownSystem.initializePinMode();
       _propSystem.initializePinModes();

        initializeServoControlRegisters();
       _propSystem.initializePropControlRegisters();

      _bleedSystem.initializePWMRegister();
    _cutdownSystem.initializePWMRegister();
       _propSystem.initializePWMOutputRegisters();
}


/**************************************************************************/
/*!
 @brief  Perform a command.
*/
/**************************************************************************/
void ALTAIR_GlobalMotorControl::performCommand(       byte                  commandByte      )
{
}


/**************************************************************************/
/*!
 @brief  Initialize the servo PWM control registers.  (The servo PWM 
         _output_ registers are initialized subsequently.)
*/
/**************************************************************************/
void ALTAIR_GlobalMotorControl::initializeServoControlRegisters(                             ) 
{

  // start up the PWM outputs
//  TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1) | _BV(WGM32) | _BV(WGM31);
//  TCCR3B = _BV(CS32);

  SERVO_MOTORS_PWMTIMER_REG_A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1) | _BV(WGM42) | _BV(WGM41);
  SERVO_MOTORS_PWMTIMER_REG_B = _BV(CS42);

}
