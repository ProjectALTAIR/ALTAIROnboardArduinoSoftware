/**************************************************************************/
/*!
    @file     ALTAIR_CutdownSystem.cpp
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

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_CutdownSystem.h"
#include "ALTAIR_MotorPWMSettings.h"

/**************************************************************************/
/*!
 @brief  Constructor.  
*/
/**************************************************************************/
ALTAIR_CutdownSystem::ALTAIR_CutdownSystem(                             ) :
    _isCutdown(                    false                                )
{
    setPWMPin(                     CUTDOWN_SERVO_PWM_PIN                ) ;
    initializeSetting(             DEFAULT_CUTDOWN_SETTING              ) ;
}

/**************************************************************************/
/*!
 @brief  (Re-)set the PWM output register.
*/
/**************************************************************************/
void ALTAIR_CutdownSystem::resetPWMRegister(                            )
{
    CUTDOWN_SERVO_PWMOUTPUT_REG  = PWM_PEDESTAL_VALUE + 2*reportSetting() ;
}

