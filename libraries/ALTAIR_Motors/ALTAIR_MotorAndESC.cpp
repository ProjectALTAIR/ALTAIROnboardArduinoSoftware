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
  _pwmValue(  0                  ) ,
  //_pwmControlValue( 0.0),
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
  // Prop Motors need correct Initialization PWM value every time they are (re)powered
  // This routine is not to first setup the registers! This is only done once
  // in ALTAIR_PropulsionSystem.initializePWMOutputRegisters()
  _pwmValue = PWM_INITIALIZATION_VALUE;
  switch(_location) {
    case portOuter:
      TCC0->CCBUF[3].reg = PWM_INITIALIZATION_VALUE;
      while (TCC0->SYNCBUSY.bit.CC3);
      break                                                              ;
    case portInner:
      TCC0->CCBUF[2].reg = PWM_INITIALIZATION_VALUE;
      while (TCC0->SYNCBUSY.bit.CC2);
      break                                                              ;
    case stbdInner:
      TCC0->CCBUF[1].reg = PWM_INITIALIZATION_VALUE;
      while (TCC0->SYNCBUSY.bit.CC1);
      break                                                              ;
    case stbdOuter:
      TCC0->CCBUF[0].reg = PWM_INITIALIZATION_VALUE;
      while (TCC0->SYNCBUSY.bit.CC0);
  }
  
  setInitialized()                                     ;
}



/**************************************************************************/
/*!
 @brief  (Re-)set the PWM output register.
*/
/**************************************************************************/
void ALTAIR_MotorAndESC::resetPWMRegister(                         )
{
  if(_pwmControlValue == 0.) { 
    _pwmValue = PWM_INITIALIZATION_VALUE;
  } else {
    _pwmValue = PWM_INITIALIZATION_VALUE + PWM_THRESHOLD_DELTA + int(PWM_STEP_DELTA * (_pwmControlValue - 1));
  }
  switch(_location) {
    case portOuter:
      //PORT_MOTOR_PWMOUTPUT_REG_A  = PWM_PEDESTAL_VALUE + 2*_pwmValue ;
      TCC0->CCBUF[3].reg = _pwmValue;
      while (TCC0->SYNCBUSY.bit.CC3);
      break                                                              ;
    case portInner:
      //PORT_MOTOR_PWMOUTPUT_REG_B  = PWM_PEDESTAL_VALUE + 2*_pwmValue ;
      TCC0->CCBUF[2].reg = _pwmValue;
      while (TCC0->SYNCBUSY.bit.CC2);
      break                                                              ;
    case stbdInner:
      //STBD_MOTOR_PWMOUTPUT_REG_B  = PWM_PEDESTAL_VALUE + 2*_pwmValue ;
      TCC0->CCBUF[1].reg = _pwmValue;
      while (TCC0->SYNCBUSY.bit.CC1);
      break                                                              ;
    case stbdOuter:
      //STBD_MOTOR_PWMOUTPUT_REG_A  = PWM_PEDESTAL_VALUE + 2*_pwmValue ;
      TCC0->CCBUF[0].reg = _pwmValue;
      while (TCC0->SYNCBUSY.bit.CC0);
  }
}




/**************************************************************************/
/*!
 @brief  Increases the PWM control value by 1.
*/
/**************************************************************************/
/*bool ALTAIR_MotorAndESC::incrementPWMControl(                            )
{
   if ( PWMControlValue() + 1.  <= PWM_MAX_SAFE_CONTROL_VALUE ) {
       _pwmControlValue++               ;
       resetPWMRegister()            ;
       return true                   ;
   } else {
       return false                  ;
   }
}*/

/**************************************************************************/
/*!
 @brief  Decreases the PWM control value by 1.
*/
/**************************************************************************/
/*bool ALTAIR_MotorAndESC::decrementPWMControl(                            )
{
   if ( PWMControlValue() - 1.  >= 0. ) {
       _pwmControlValue--               ;
       resetPWMRegister()            ;  
       return true                   ;
   } else {
       return false                  ;
   }
}*/

/**************************************************************************/
/*!
 @brief  Increases the PWM control value by 0.5.
*/
/**************************************************************************/
/*bool ALTAIR_MotorAndESC::halfIncrementPWMControl(                        )
{
   if ( PWMControlValue() + 0.5  <= PWM_MAX_SAFE_CONTROL_VALUE ) {
       _pwmControlValue          += 0.5 ;
       resetPWMRegister()            ;  
       return true                   ;
   } else {
       return false                  ;
   }
}*/

/**************************************************************************/
/*!
 @brief  Decreases the PWM control value by 0.5.
*/
/**************************************************************************/
/*bool ALTAIR_MotorAndESC::halfDecrementPWMControl(                        )
{
   if ( PWMControlValue() - 0.5  >= 0. ) {
       _pwmControlValue          -= 0.5 ;
       resetPWMRegister()            ;  
       return true                   ;
   } else {
       return false                  ;
   }
}*/

/**************************************************************************/
/*!
 @brief  Sets the PWM control value to a new setting between 0 and 
         PWM_MAX_SAFE_CONTROL_VALUE.
*/
/**************************************************************************/
bool ALTAIR_MotorAndESC::setPWMControlValueTo( float newPWMControlValue   )
{
   if ( newPWMControlValue >= 0. && newPWMControlValue <= PWM_MAX_SAFE_CONTROL_VALUE ) {
       _pwmControlValue           = newPWMControlValue  ;
       resetPWMRegister()                         ;  
       return true                                ;
   } else {
       return false                               ;
   }
}

