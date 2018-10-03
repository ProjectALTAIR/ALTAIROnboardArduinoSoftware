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
ALTAIR_GlobalMotorControl::ALTAIR_GlobalMotorControl(                                        ) :
                          _bleedSystem(                BLEEDVALVE_SERVO_POS_ADC_PIN          ),
                          _cutdownSystem(              CUTDOWN_SERVO_POS_ADC_PIN             )
{
}

/**************************************************************************/
/*!
 @brief  Fully initialize all ALTAIR motors.
*/
/**************************************************************************/
bool ALTAIR_GlobalMotorControl::initializeAllMotors(                                         )
{
      _bleedSystem.initializePinMode()              ;
    _cutdownSystem.initializePinMode()              ;
       _propSystem.initializePinModes()             ;

        initializeServoControlRegisters()           ;
       _propSystem.initializePropControlRegisters() ;

      _bleedSystem.initializePWMRegister()          ;
    _cutdownSystem.initializePWMRegister()          ;
       _propSystem.initializePWMOutputRegisters()   ;

        return     true                             ;
}


/**************************************************************************/
/*!
 @brief  Perform a command.
*/
/**************************************************************************/
void ALTAIR_GlobalMotorControl::performCommand(       byte                  commandByte      )
{
  switch(commandByte) {
    case 'A':
      _propSystem.axleRotServo()->incrementSetting();
       break;
    case 'a':
      _propSystem.axleRotServo()->decrementSetting();
       break;
    case 'B':
      _bleedSystem.incrementSetting();
       break;
    case 'b':
      _bleedSystem.decrementSetting();
       break;
    case 'C':
      _cutdownSystem.incrementSetting();
       break;
    case 'c':
      _cutdownSystem.decrementSetting();
       break;
    case 'D':
      _propSystem.portOuterMotor()->incrementPower();
       break;
    case 'd':
      _propSystem.portOuterMotor()->decrementPower();
       break;
    case 'E':
      _propSystem.portInnerMotor()->incrementPower();
       break;
    case 'e':
      _propSystem.portInnerMotor()->decrementPower();
       break;
    case 'F':
      _propSystem.stbdInnerMotor()->incrementPower();
       break;
    case 'f':
      _propSystem.stbdInnerMotor()->decrementPower();
       break;
    case 'G':
      _propSystem.stbdOuterMotor()->incrementPower();
       break;
    case 'g':
      _propSystem.stbdOuterMotor()->decrementPower();
       break;
    case 'H':
      _propSystem.halfIncrementPower();
       break;
    case 'h':
      _propSystem.halfDecrementPower();
       break;
    case 'U':
      _propSystem.incrementPower();
       break;
    case 'u':
      _propSystem.decrementPower();
       break;
    case 'X':
    case 'x':
      _propSystem.shutDownAllProps();
       break;
    default :
       break;
  }
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
