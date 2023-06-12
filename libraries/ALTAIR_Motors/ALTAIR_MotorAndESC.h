/**************************************************************************/
/*!
    @file     ALTAIR_MotorAndESC.h
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

    Justin Albert  jalbert@uvic.ca     began on 1 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef   ALTAIR_MotorAndESC_h
#define   ALTAIR_MotorAndESC_h

#include "Arduino.h"
#include "ALTAIR_RPMSensor.h"
#include "ALTAIR_CurrentSensor.h"
#include "ALTAIR_TempSensor.h"


typedef enum { portOuter ,
               portInner ,
               stbdInner ,
               stbdOuter } location_t;

class ALTAIR_MotorAndESC {
  public:

    ALTAIR_MotorAndESC()                                                           ;

    //void                     initializePinMode()     { pinMode(_pwmPin, OUTPUT)    ; }  // Not needed on GrandCentral
    void                     initializePWMRegister()                               ;
    bool                     isInitialized()          { return  _isInitialized      ; }
    bool                     isRunning()              { return (_pwmControlValue > 0.) ; }

    float                    PWMControlValue()        { return  _pwmControlValue    ; } // Control setting can be from 0 through PWM_MAX_SAFE_CONTROL_VALUE.
    int                      PWMValue()               { return _pwmValue;}
    
      // Incremential Control is implemented in Propulsion System Class
    //bool                     incrementPWMControl()                                      ;   // Increase power setting by 1.    Returns true if successful.
    //bool                     decrementPWMControl()                                      ;   // Decrease power setting by 1.    Returns true if successful.
    //bool                     halfIncrementPWMControl()                                  ;   // Increase power setting by 0.5.  Returns true if successful.
    //bool                     halfDecrementPWMControl()                                  ;   // Decrease power setting by 0.5.  Returns true if successful.
    
    bool                     setPWMControlValueTo( float newPWMControlValue )                   ;   // Returns true if successful.

    //ALTAIR_RPMSensor&        rpmSensor()             { return _rpmSensor           ; }
    //ALTAIR_CurrentSensor&    currentSensor()         { return _currentSensor       ; }
    //ALTAIR_TempSensor&       motorTempSensor()       { return _tempSensor[0]       ; }
    //ALTAIR_TempSensor&       escTempSensor()         { return _tempSensor[1]       ; }

    void                     makePortOuter()                                       ;
    void                     makePortInner()                                       ;
    void                     makeStbdInner()                                       ;
    void                     makeStbdOuter()                                       ;

  protected:
    void                     setInitialized()        { _isInitialized  = true      ; }
    void                     resetPWMRegister()                                    ;

  private:
    int                     _pwmValue                                         ;
    float                   _pwmControlValue;
    bool                    _isInitialized                                        ;
    
    //ALTAIR_RPMSensor        _rpmSensor                                            ;
    //ALTAIR_CurrentSensor    _currentSensor                                        ;
    //ALTAIR_TempSensor       _tempSensor[2]                                        ;

    byte                    _pwmPin                                               ;
    location_t              _location                                             ;

};
#endif    //   ifndef ALTAIR_MotorAndESC_h
