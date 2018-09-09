/**************************************************************************/
/*!
    @file     ALTAIR_MotorAndESC.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class of each ALTAIR motor plus ESC (electronic speed
    controller) object, of which ALTAIR has four (corresponding to the port
    outer and inner, and the starboard outer and inner, motors).  Each motor
    has an optical RPM sensor (so ALTAIR has a total of 4 RPM sensors); each
    motor/ESC pair has a current sensor (so ALTAIR has a total of 4 current
    sensors); and each motor and each ESC has its own temperature sensor (so
    ALTAIR has a total of 8 propulsion system temperature sensors).  Four
    (and only four) objects of this ALTAIR_MotorAndESC class should end up
    instantiated; they will all be instantiated by the
    ALTAIR_PropulsionSystem object upon the singleton instantiation of that
    ALTAIR_PropulsionSystem object.

    Justin Albert  jalbert@uvic.ca     began on 31 Aug. 2018

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_MotorAndESC.h"
#include "ALTAIR_MotorPWMSettings.h"


/**************************************************************************/
/*!
 @brief  Constructor.  The necessary values are set in the makePortOuter()
         or makeStbdInner() or etc methods below.
*/
/**************************************************************************/
ALTAIR_MotorAndESC::ALTAIR_MotorAndESC() :
  _powerSetting(  0.0                  ) ,
  _isInitialized(   false              )
{
}

/**************************************************************************/
/*!
 @brief  Sets the pwmPin and PWM register pointer values to their correct 
         settings.
*/
/**************************************************************************/
void ALTAIR_MotorAndESC::makePortOuter(                             )
{
  _pwmPin                        = PORT_OUTER_MOTOR_PWM_PIN ;
  _location                      = portOuter                ;
}

/**************************************************************************/
/*!
 @brief  Sets the pwmPin and PWM register pointer values to their correct 
         settings.
*/
/**************************************************************************/
void ALTAIR_MotorAndESC::makePortInner(                             )
{
  _pwmPin                        = PORT_INNER_MOTOR_PWM_PIN ;
  _location                      = portInner                ;
}

/**************************************************************************/
/*!
 @brief  Sets the pwmPin and PWM register pointer values to their correct 
         settings.
*/
/**************************************************************************/
void ALTAIR_MotorAndESC::makeStbdInner(                             )
{
  _pwmPin                        = STBD_INNER_MOTOR_PWM_PIN ;
  _location                      = stbdInner                ;
}

/**************************************************************************/
/*!
 @brief  Sets the pwmPin and PWM register pointer values to their correct 
         settings.
*/
/**************************************************************************/
void ALTAIR_MotorAndESC::makeStbdOuter(                             )
{
  _pwmPin                        = STBD_OUTER_MOTOR_PWM_PIN ;
  _location                      = stbdOuter                ;
}

/**************************************************************************/
/*!
 @brief  Initialize the PWM output register after power-on, after the 
         pin mode is initialized, and after the timer-counter control
         registers are initialized.
*/
/**************************************************************************/
void ALTAIR_MotorAndESC::initializePWMRegister(                     )
{
  resetPWMRegister()                                   ;
  setInitialized()                                     ;
}



/**************************************************************************/
/*!
 @brief  (Re-)set the PWM output register.
*/
/**************************************************************************/
void ALTAIR_MotorAndESC::resetPWMRegister(                         )
{
  switch(_location) {
    case portOuter:
      PORT_MOTOR_PWMOUTPUT_REG_A  = PWM_PEDESTAL_VALUE + 2*_powerSetting ;
      break                                                              ;
    case portInner:
      PORT_MOTOR_PWMOUTPUT_REG_B  = PWM_PEDESTAL_VALUE + 2*_powerSetting ;
      break                                                              ;
    case stbdInner:
      STBD_MOTOR_PWMOUTPUT_REG_B  = PWM_PEDESTAL_VALUE + 2*_powerSetting ;
      break                                                              ;
    case stbdOuter:
      STBD_MOTOR_PWMOUTPUT_REG_A  = PWM_PEDESTAL_VALUE + 2*_powerSetting ;
  }
}




/**************************************************************************/
/*!
 @brief  Increases the power setting by 1.
*/
/**************************************************************************/
bool ALTAIR_MotorAndESC::incrementPower(                            )
{
   if ( powerSetting() + 1.  <= MAX_SAFE_PROPMOTOR_SETTING ) {
       _powerSetting++               ;
       resetPWMRegister()            ;
       return true                   ;
   } else {
       return false                  ;
   }
}

/**************************************************************************/
/*!
 @brief  Decreases the power setting by 1.
*/
/**************************************************************************/
bool ALTAIR_MotorAndESC::decrementPower(                            )
{
   if ( powerSetting() - 1.  >= 0. ) {
       _powerSetting--               ;
       resetPWMRegister()            ;  
       return true                   ;
   } else {
       return false                  ;
   }
}

/**************************************************************************/
/*!
 @brief  Increases the power setting by 0.5.
*/
/**************************************************************************/
bool ALTAIR_MotorAndESC::halfIncrementPower(                        )
{
   if ( powerSetting() + 0.5  <= MAX_SAFE_PROPMOTOR_SETTING ) {
       _powerSetting          += 0.5 ;
       resetPWMRegister()            ;  
       return true                   ;
   } else {
       return false                  ;
   }
}

/**************************************************************************/
/*!
 @brief  Decreases the power setting by 0.5.
*/
/**************************************************************************/
bool ALTAIR_MotorAndESC::halfDecrementPower(                        )
{
   if ( powerSetting() - 0.5  >= 0. ) {
       _powerSetting          -= 0.5 ;
       resetPWMRegister()            ;  
       return true                   ;
   } else {
       return false                  ;
   }
}

/**************************************************************************/
/*!
 @brief  Sets the power to a new setting between 0 and 
         MAX_SAFE_PROPMOTOR_SETTING.
*/
/**************************************************************************/
bool ALTAIR_MotorAndESC::setPowerTo( float newPowerSetting   )
{
   if ( newPowerSetting >= 0. && newPowerSetting <= MAX_SAFE_PROPMOTOR_SETTING ) {
       _powerSetting           = newPowerSetting  ;
       resetPWMRegister()                         ;  
       return true                                ;
   } else {
       return false                               ;
   }
}

