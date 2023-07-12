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
ALTAIR_PropulsionSystem::ALTAIR_PropulsionSystem(                                        )
{
    (_motorAndESC[0]).makePortOuter();
    (_motorAndESC[1]).makePortInner();
    (_motorAndESC[2]).makeStbdInner();
    (_motorAndESC[3]).makeStbdOuter();
}
/* Constructor when propAxleRotServo is attached
ALTAIR_PropulsionSystem::ALTAIR_PropulsionSystem(                                        ) :
     _propAxleRotServo(                               PROPAXLEROT_SERVO_POS_ADC_PIN      )
{
    (_motorAndESC[0]).makePortOuter();
    (_motorAndESC[1]).makePortInner();
    (_motorAndESC[2]).makeStbdInner();
    (_motorAndESC[3]).makeStbdOuter();
}
*/

/**************************************************************************/
/*!
 @brief  Initialize the propulsion system control pin modes after 
         power-on (within the setup routine).
*/
/**************************************************************************/
/*  //Not needed with GRAND CENTRAL
void ALTAIR_PropulsionSystem::initializePinModes(                                        )
{
    _propAxleRotServo.initializePinMode();
    for (int i = 0; i < 4; ++i) (_motorAndESC[i]).initializePinMode();
}
*/
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
void ALTAIR_PropulsionSystem::initializePropControlRegisters(                            )
{
    /* OLD ATMEGA CODE
    PORT_MOTOR_PWMTIMER_REG_A = _BV(COM5A1) | _BV(COM5B1) | _BV(WGM52) | _BV(WGM51);
    PORT_MOTOR_PWMTIMER_REG_B = _BV(CS52);                                    
    STBD_MOTOR_PWMTIMER_REG_A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM12) | _BV(WGM11);
    STBD_MOTOR_PWMTIMER_REG_B = _BV(CS12);
    */

    // GRAND CENTRAL PWM SETUP ROUTINE
    // Set up the generic clock (GCLK7) to clock timer TCC0
    // 14.6.2.1 Initializing the Generator Control register GENCTRLn with a single 32-bit write
    // 14.7 Register Summary: 7:0 SRC; 8:15 GEN (including the DIVSEL bit); 23:16 DIV[7:0]; 31:24 DIV[15:8]
    // If GENCTRLn.DIVSEL=0 and GENCTRLn.DIV is either 0 or 1, the output clock will be undivided.
    // 14.8.3 Detailed description of every bit in the GENCTRLn register
    GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 48MHz clock source by divisor 1: 48MHz/1 = 48MHz
                            GCLK_GENCTRL_IDC |          // Improve Duty Cycle IDC sets the clock source duty cycle to 50/50 HIGH/LOW
                            GCLK_GENCTRL_GENEN |        // Enable GCLK7
                            GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
    while (GCLK->SYNCBUSY.bit.GENCTRL7);               // Wait for synchronization  

    // 14.6.3 Set up Peripheral Clock by selecting the previously set up GCLK7 and mapping it to TCC0
    GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC0 peripheral channel (see Table 14-9 on p. 157 of SAMD data sheet for mapping 25 -> TCC0)
                            GCLK_PCHCTRL_GEN_GCLK7;    // Connect generic clock 7 GCLK7

    // Enable the peripheral multiplexer on pin D33  
    // Set the D33 (PORT_PA20) peripheral multiplexer to peripheral (even port number -- 20 is even) E(6): TCC0, Channel 0. 
    // (The analogous operation for odd port numbers would be PMUXO -- for more details, see p. 825 of SAMD data sheet.  
    //  Note that E(6) of PMUXE, per p. 825 of the data sheet, is peripheral function G: for that, see row containing "PB12"
    //  near middle of p. 33 of the SAMD data sheet: you'll find TCC0/WO[0] (i.e. TCC0, Channel 0) there.)
    PORT->Group[g_APinDescription[33].ulPort].PINCFG[g_APinDescription[33].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[33].ulPort].PMUX[g_APinDescription[33].ulPin >> 1].reg |= PORT_PMUX_PMUXE(6);

    // Same operation for D31 (POR_PA22): TCC0, Channel 2 
    PORT->Group[g_APinDescription[31].ulPort].PINCFG[g_APinDescription[31].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[31].ulPort].PMUX[g_APinDescription[31].ulPin >> 1].reg |= PORT_PMUX_PMUXE(6);

    // Uneven ports
    // Set up for D32 (POR_PA21): TCC0, Channel 1 
    PORT->Group[g_APinDescription[32].ulPort].PINCFG[g_APinDescription[32].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[32].ulPort].PMUX[g_APinDescription[32].ulPin >> 1].reg |= PORT_PMUX_PMUXO(6);

    // Set up for D30 (POR_PA23): TCC0, Channel 3 
    PORT->Group[g_APinDescription[30].ulPort].PINCFG[g_APinDescription[30].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[30].ulPort].PMUX[g_APinDescription[30].ulPin >> 1].reg |= PORT_PMUX_PMUXO(6);
    

    // DEFINE PWM SIGNAL
    // Prescaler from 48MHz GCLK7 to 6MHz
    TCC0->CTRLA.reg = TC_CTRLA_PRESCALER_DIV8 |        // Set prescaler to 8, 48MHz/8 = 6MHz
                        TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock                 

    // Set-up TCC0 timer for Normal PWM (single slope)
    TCC0->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC0 timer for Normal (single slope) PWM mode (NPWM)
    while (TCC0->SYNCBUSY.bit.WAVE)                    // Wait for synchronization

    // Define PWM Period to 50 Hz by deviding the 6MHz
    TCC0->PER.reg = PWM_PERIOD;                        // Set-up the PER (period) register 50Hz PWM (6MHz/119999 ~= 6MHz/120000 = 50Hz)
    while (TCC0->SYNCBUSY.bit.PER);                    // Wait for synchronization

    // Define Duty cycle with comparison to the PWM Period
    // Initialization value for the motors when they power up. This defines their operating range. Rotation starts at ~300 more than this initialization value
    TCC0->CC[0].reg = PWM_INITIALIZATION_VALUE;        // Set-up the CC (counter compare), channel 0 register
    while (TCC0->SYNCBUSY.bit.CC0);                    // Wait for synchronization
    TCC0->CC[1].reg = PWM_INITIALIZATION_VALUE;        // Channels 1..3
    while (TCC0->SYNCBUSY.bit.CC1);                    
    TCC0->CC[2].reg = PWM_INITIALIZATION_VALUE;        
    while (TCC0->SYNCBUSY.bit.CC2);                    
    TCC0->CC[3].reg = PWM_INITIALIZATION_VALUE;        
    while (TCC0->SYNCBUSY.bit.CC3);                    

    // Enable
    TCC0->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC0
    while (TCC0->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization

}

/**************************************************************************/
/*!
 @brief  Initialize the propulsion system PWM output registers BEFORE
         power-on of motors!, and after the PWM control registers are initialized 
         (within the setup routine).
         This routine is called anytime during runtime before Prop Motors are
         turned on, in order to ensure correct control!
*/
/**************************************************************************/
void ALTAIR_PropulsionSystem::initializePWMOutputRegisters(                              )
{
    //_propAxleRotServo.initializePWMRegister();
    for (int i = 0; i < 4; i++) (_motorAndESC[i]).initializePWMRegister();
}

/**************************************************************************/
/*!
 @brief  Returns true _only_ if _all_ of the motors are initialized.
*/
/**************************************************************************/
bool ALTAIR_PropulsionSystem::isInitialized(                                             )
{
    return ( portOuterMotor()->isInitialized() && 
             portInnerMotor()->isInitialized() &&
             stbdInnerMotor()->isInitialized() &&
             stbdOuterMotor()->isInitialized()   );
    
    // When axleRotServo is connected to the system!
    /*return ( axleRotServo()->isInitialized() &&
             portOuterMotor()->isInitialized() && 
             portInnerMotor()->isInitialized() &&
             stbdInnerMotor()->isInitialized() &&
             stbdOuterMotor()->isInitialized()   );*/
}

/**************************************************************************/
/*!
 @brief  Returns true if _any_ of the propulsion motors is spinning.  (Note:
         does NOT return true [but rather false] if the axle rotation servo
         is rotating, however none of the propulsion motors is spinning.)
*/
/**************************************************************************/
bool ALTAIR_PropulsionSystem::isRunning(                                                 )
{
    return ( portOuterMotor()->isRunning()     || 
             portInnerMotor()->isRunning()     ||
             stbdInnerMotor()->isRunning()     ||
             stbdOuterMotor()->isRunning()       );
}

/**************************************************************************/
/*!
 @brief  Specific Motor can be selected for individual Control
*/
/**************************************************************************/
void ALTAIR_PropulsionSystem::selectIndividualMotor(ALTAIR_MotorAndESC* pSelectedMotor )
{
    _pSelectedMotor = pSelectedMotor;
}

/**************************************************************************/
/*!
 @brief  Increment the PWM control depending on the selected control Mode.
*/
/**************************************************************************/
bool ALTAIR_PropulsionSystem::incrementPWMControl( controlMode controlMode )
{
    switch (controlMode)
    {
    case individual:
        return ( changePWMControlValue(selectedMotor(),  1.)    );
        break;
    case outerPair:
        return ( changePWMControlValue(portOuterMotor(), 1.) &&
                 changePWMControlValue(stbdOuterMotor(), 1.)    );
        break;
    case innerPair:
        return ( changePWMControlValue(portInnerMotor(), 1.) &&
                 changePWMControlValue(stbdInnerMotor(), 1.)    );
        break;
    case combined:
        return ( changePWMControlValue(portOuterMotor(), 1.) &&
                 changePWMControlValue(portInnerMotor(), 1.) &&
                 changePWMControlValue(stbdInnerMotor(), 1.) &&   
                 changePWMControlValue(stbdOuterMotor(), 1.)    );
        break;
    default:
        return false;
        break;
    }
}

/**************************************************************************/
/*!
 @brief  Decrement the PWM control depending on the selected control Mode.
*/
/**************************************************************************/
bool ALTAIR_PropulsionSystem::decrementPWMControl( controlMode controlMode )
{
    switch (controlMode)
    {
    case individual:
        return ( changePWMControlValue(selectedMotor(),  -1.)    );
        break;
    case outerPair:
        return ( changePWMControlValue(portOuterMotor(), -1.) &&
                 changePWMControlValue(stbdOuterMotor(), -1.)    );
        break;
    case innerPair:
        return ( changePWMControlValue(portInnerMotor(), -1.) &&
                 changePWMControlValue(stbdInnerMotor(), -1.)    );
        break;
    case combined:
        return ( changePWMControlValue(portOuterMotor(), -1.) &&
                 changePWMControlValue(portInnerMotor(), -1.) &&
                 changePWMControlValue(stbdInnerMotor(), -1.) &&   
                 changePWMControlValue(stbdOuterMotor(), -1.)    );
        break;
    default:
        return false;
        break;
    }
}

/**************************************************************************/
/*!
 @brief  Half Increment the PWM control depending on the selected control Mode.
*/
/**************************************************************************/
bool ALTAIR_PropulsionSystem::halfIncrementPWMControl( controlMode controlMode )
{
    switch (controlMode)
    {
    case individual:
        return ( changePWMControlValue(selectedMotor(),  0.5)    );
        break;
    case outerPair:
        return ( changePWMControlValue(portOuterMotor(), 0.5) &&
                 changePWMControlValue(stbdOuterMotor(), 0.5)    );
        break;
    case innerPair:
        return ( changePWMControlValue(portInnerMotor(), 0.5) &&
                 changePWMControlValue(stbdInnerMotor(), 0.5)    );
        break;
    case combined:
        return ( changePWMControlValue(portOuterMotor(), 0.5) &&
                 changePWMControlValue(portInnerMotor(), 0.5) &&
                 changePWMControlValue(stbdInnerMotor(), 0.5) &&   
                 changePWMControlValue(stbdOuterMotor(), 0.5)    );
        break;
    default:
        return false;
        break;
    }
}


/**************************************************************************/
/*!
 @brief  Half Decrement the PWM control depending on the selected control Mode.
*/
/**************************************************************************/
bool ALTAIR_PropulsionSystem::halfDecrementPWMControl( controlMode controlMode )
{
    switch (controlMode)
    {
    case individual:
        return ( changePWMControlValue(selectedMotor(),  -0.5)    );
        break;
    case outerPair:
        return ( changePWMControlValue(portOuterMotor(), -0.5) &&
                 changePWMControlValue(stbdOuterMotor(), -0.5)    );
        break;
    case innerPair:
        return ( changePWMControlValue(portInnerMotor(), -0.5) &&
                 changePWMControlValue(stbdInnerMotor(), -0.5)    );
        break;
    case combined:
        return ( changePWMControlValue(portOuterMotor(), -0.5) &&
                 changePWMControlValue(portInnerMotor(), -0.5) &&
                 changePWMControlValue(stbdInnerMotor(), -0.5) &&   
                 changePWMControlValue(stbdOuterMotor(), -0.5)    );
        break;
    default:
        return false;
        break;
    }
}


/**************************************************************************/
/*!
 @brief  Set the PWM control depending on the selected control mode to some Control Value.
*/
/**************************************************************************/
bool ALTAIR_PropulsionSystem::setPWMControlTo( controlMode controlMode, float newPWMControlValue )
{
    switch (controlMode)
    {
    case individual:
        return ( changePWMControlValue(selectedMotor(),  newPWMControlValue - selectedMotor()->PWMControlValue())     );
        break;
    case outerPair:
        return ( changePWMControlValue(portOuterMotor(), newPWMControlValue - portOuterMotor()->PWMControlValue()) &&
                 changePWMControlValue(stbdOuterMotor(), newPWMControlValue - stbdOuterMotor()->PWMControlValue())    );
        break;
    case innerPair:
        return ( changePWMControlValue(portInnerMotor(), newPWMControlValue - portInnerMotor()->PWMControlValue()) &&
                 changePWMControlValue(stbdInnerMotor(), newPWMControlValue - stbdInnerMotor()->PWMControlValue())    );
        break;
    case combined:
        return ( changePWMControlValue(portOuterMotor(), newPWMControlValue - portOuterMotor()->PWMControlValue()) &&
                 changePWMControlValue(portInnerMotor(), newPWMControlValue - portInnerMotor()->PWMControlValue()) &&
                 changePWMControlValue(stbdInnerMotor(), newPWMControlValue - stbdInnerMotor()->PWMControlValue()) &&   
                 changePWMControlValue(stbdOuterMotor(), newPWMControlValue - stbdOuterMotor()->PWMControlValue())    );
        break;
    default:
        return false;
        break;
    }
}


/**************************************************************************/
/*!
 @brief  Change the PWM control of a Prop Motor by deltaPWMControlValue.
*/
/**************************************************************************/
bool ALTAIR_PropulsionSystem::changePWMControlValue(ALTAIR_MotorAndESC* PropMotor ,float deltaPWMControlValue )
{
    return ( PropMotor->setPWMControlValueTo( PropMotor->PWMControlValue() + deltaPWMControlValue));
}


/**************************************************************************/
/*!
 @brief OLD 
        Change the power setting of all prop motors by deltaPower.
*/
/**************************************************************************/
/*
bool ALTAIR_PropulsionSystem::changePower(              float                 deltaPower )
{
    return ( portOuterMotor()->setPowerTo( portOuterMotor()->powerSetting() + deltaPower ) &&
             portInnerMotor()->setPowerTo( portInnerMotor()->powerSetting() + deltaPower ) &&
             stbdInnerMotor()->setPowerTo( stbdInnerMotor()->powerSetting() + deltaPower ) &&
             stbdOuterMotor()->setPowerTo( stbdOuterMotor()->powerSetting() + deltaPower )    );
}
*/

/**************************************************************************/
/*!
 @brief  Change the power setting of all prop motors to 0.
*/
/**************************************************************************/
bool ALTAIR_PropulsionSystem::shutDownAllProps(                                          )
{
    return ( portOuterMotor()->setPWMControlValueTo( 0. ) &&
             portInnerMotor()->setPWMControlValueTo( 0. ) &&
             stbdInnerMotor()->setPWMControlValueTo( 0. ) &&
             stbdOuterMotor()->setPWMControlValueTo( 0. )    );
}

