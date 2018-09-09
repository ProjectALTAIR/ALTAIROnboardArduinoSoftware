/**************************************************************************/
/*!
    @file     ALTAIR_ServoMotor.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the base class for the three ALTAIR servo motors: the
    propulsion Axle rotation servo motor (A); the balloon helium Bleed
    valve servo motor (B); and the Cutdown and parafoil steering servo
    motor (C).  Servo A is a Corona CS238MG (actually, its older product
    code is CS-238HV, but I believe it is the same device:
    https://hobbyking.com/en_us/corona-cs238mg-metal-gear-servo-4-6kg-0-14sec-22g.html
    ); Servo B is a Corona DS538HV
    https://hobbyking.com/en_us/corona-ds538hv-digital-metal-gear-servo-8kg-0-12sec-58g.html
    ; and servo C is a HobbyKing brand SW5513-6MA sail winch servo:
    https://hobbyking.com/en_us/sail-winch-servo-13kg-0-7sec-360deg-55g.html
    .  All three servos have been modified to include an analogue
    encoder voltage output (from the potentiometer inside each servo)
    to report the servo's position via that analogue voltage.

    Justin Albert  jalbert@uvic.ca     began on 2 Sep. 2018

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_ServoMotor.h"


/**************************************************************************/
/*!
 @brief  Constructor.  
*/
/**************************************************************************/
ALTAIR_ServoMotor::ALTAIR_ServoMotor() :
     _isInitialized(   false ),
     _setting      (  0.     ),
     _position     (  0.     )
{
}

/**************************************************************************/
/*!
 @brief  Determine and report the present position of the servo.
*/
/**************************************************************************/
float ALTAIR_ServoMotor::determinePosition() 
{
    return 3.5;    // implement this later
}

/**************************************************************************/
/*!
 @brief  Increases the setting by 1.
*/
/**************************************************************************/
bool ALTAIR_ServoMotor::incrementSetting(                            )
{
   if ( reportSetting() + 1.  <= maxSafeSetting() ) {
       _setting++                    ;
       resetPWMRegister()            ;
       return true                   ;
   } else {
       return false                  ;
   }
}

/**************************************************************************/
/*!
 @brief  Decreases the setting by 1.
*/
/**************************************************************************/
bool ALTAIR_ServoMotor::decrementSetting(                            )
{
   if ( reportSetting() - 1.  >= minSafeSetting() ) {
       _setting--                    ;
       resetPWMRegister()            ;
       return true                   ;
   } else {
       return false                  ;
   }
}

/**************************************************************************/
/*!
 @brief  Increases the setting by 0.5.
*/
/**************************************************************************/
bool ALTAIR_ServoMotor::halfIncrementSetting(                        )
{
   if ( reportSetting() + 0.5 <= maxSafeSetting() ) {
       _setting               += 0.5 ;
       resetPWMRegister()            ;
       return true                   ;
   } else {
       return false                  ;
   }
}

/**************************************************************************/
/*!
 @brief  Decreases the setting by 0.5.
*/
/**************************************************************************/
bool ALTAIR_ServoMotor::halfDecrementSetting(                        )
{
   if ( reportSetting() - 0.5 >= minSafeSetting() ) {
       _setting               -= 0.5 ;
       resetPWMRegister()            ;
       return true                   ;
   } else {
       return false                  ;
   }
}

/**************************************************************************/
/*!
 @brief  Move the servo to a new setting.
*/
/**************************************************************************/
bool ALTAIR_ServoMotor::setSettingTo(   float  newSetting  )
{
   if ( newSetting >= minSafeSetting() && newSetting <= maxSafeSetting() ) {
       _setting            = newSetting  ;
       resetPWMRegister()                ;
       return true                       ;
   } else {
       return false                      ;
   }
}


/**************************************************************************/
/*!
 @brief  Initialize the Arduino Mega 2560 control pin mode after
         power-on (within the setup routine).
*/
/**************************************************************************/
void ALTAIR_ServoMotor::initializePinMode(                                   )
{
    pinMode(_pwmPin, OUTPUT)    ;
}

/**************************************************************************/
/*!
 @brief  Initialize the PWM output register after power-on, after the
         pin mode is initialized, and after the timer-counter control
         registers are initialized.
*/
/**************************************************************************/
void ALTAIR_ServoMotor::initializePWMRegister(                     )
{
     resetPWMRegister()         ;
     setInitialized()           ;
}

