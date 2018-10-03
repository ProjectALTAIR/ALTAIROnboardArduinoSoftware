/**************************************************************************/
/*!
    @file     ALTAIR_MotorPWMSettings.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This class contains all the #defines for all the servo and propulsion
    motor fixed connections; the PWM registers; and the max, min, and 
    default settings.

    Justin Albert  jalbert@uvic.ca     began on 1 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef   ALTAIR_MotorPWMSettings_h
#define   ALTAIR_MotorPWMSettings_h


#define   PORT_OUTER_MOTOR_PWM_PIN      45         // The pulse-width modulation digital output pin of  
#define   PORT_INNER_MOTOR_PWM_PIN      46         // the Arduino Mega 2560 that controls a given motor.
#define   STBD_INNER_MOTOR_PWM_PIN      11
#define   STBD_OUTER_MOTOR_PWM_PIN      12

#define   PROPAXLEROT_SERVO_PWM_PIN      6
#define   BLEEDVALVE_SERVO_PWM_PIN       7
#define   CUTDOWN_SERVO_PWM_PIN          8

#define   PROPAXLEROT_SERVO_POS_ADC_PIN A6         // FIX THESE VALUES!!!!!
#define   BLEEDVALVE_SERVO_POS_ADC_PIN  A7
#define   CUTDOWN_SERVO_POS_ADC_PIN     A8

#define   DEFAULT_PROPAXLEROT_SETTING    6.        // The default servo settings.
#define   DEFAULT_BLEEDVALVE_SETTING    15.
#define   DEFAULT_CUTDOWN_SETTING        6.

#define   MAX_SAFE_PROPAXLEROT_SETTING  15.
#define   MAX_SAFE_BLEEDVALVE_SETTING   16.
#define   MAX_SAFE_CUTDOWN_SETTING      15.

#define   MIN_SAFE_PROPAXLEROT_SETTING   0.
#define   MIN_SAFE_BLEEDVALVE_SETTING    0.
#define   MIN_SAFE_CUTDOWN_SETTING       0.

#define   MAX_SAFE_PROPMOTOR_SETTING     2.5       // A very important floating-point number btw 0 and 10.

#define   PWM_PEDESTAL_VALUE            34         // If the content of the PWM output register is increased
                                                   // above this value, then the motor starts to spin.

#define   PORT_MOTOR_PWMTIMER_REG_A        TCCR5A  // ATmega 2560 timer-counter control register 5A, etc.
#define   PORT_MOTOR_PWMTIMER_REG_B        TCCR5B
#define   STBD_MOTOR_PWMTIMER_REG_A        TCCR1A
#define   STBD_MOTOR_PWMTIMER_REG_B        TCCR1B

#define   SERVO_MOTORS_PWMTIMER_REG_A      TCCR4A  // ATmega 2560 timer-counter control register 4A, etc.
#define   SERVO_MOTORS_PWMTIMER_REG_B      TCCR4B

#define   PORT_MOTOR_PWMOUTPUT_REG_A        OCR5A  // ATmega 2560 output control register 5A, etc.
#define   PORT_MOTOR_PWMOUTPUT_REG_B        OCR5B
#define   STBD_MOTOR_PWMOUTPUT_REG_A        OCR1A
#define   STBD_MOTOR_PWMOUTPUT_REG_B        OCR1B

#define   PROPAXLEROT_SERVO_PWMOUTPUT_REG   OCR4A  // ATmega 2560 output control register 4A, etc.
#define   BLEEDVALVE_SERVO_PWMOUTPUT_REG    OCR4B 
#define   CUTDOWN_SERVO_PWMOUTPUT_REG       OCR4C


#endif    //   ifndef ALTAIR_MotorPWMSettings_h
