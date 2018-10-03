/**************************************************************************/
/*!
    @file     ALTAIR_PropAxleRotServo.cpp
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

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_PropAxleRotServo.h"
#include "ALTAIR_MotorPWMSettings.h"

/**************************************************************************/
/*!
 @brief  Constructor.  
*/
/**************************************************************************/
ALTAIR_PropAxleRotServo::ALTAIR_PropAxleRotServo( byte              posADCPin ) :
                         ALTAIR_ServoMotor(                         posADCPin )
{
    setPWMPin(                                    PROPAXLEROT_SERVO_PWM_PIN   ) ;
    initializeSetting(                            DEFAULT_PROPAXLEROT_SETTING ) ;
}

/**************************************************************************/
/*!
 @brief  Return the maximum safe setting.
*/
/**************************************************************************/
float ALTAIR_PropAxleRotServo::maxSafeSetting(                              )
{
    return                         MAX_SAFE_PROPAXLEROT_SETTING               ;
}

/**************************************************************************/
/*!
 @brief  Return the minimum safe setting.
*/
/**************************************************************************/
float ALTAIR_PropAxleRotServo::minSafeSetting(                              )
{
    return                         MIN_SAFE_PROPAXLEROT_SETTING               ;
}

/**************************************************************************/
/*!
 @brief  (Re-)set the PWM output register.
*/
/**************************************************************************/
void ALTAIR_PropAxleRotServo::resetPWMRegister(                             )
{
    PROPAXLEROT_SERVO_PWMOUTPUT_REG  = PWM_PEDESTAL_VALUE + 2*reportSetting() ;
}

