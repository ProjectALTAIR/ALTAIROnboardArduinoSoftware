
#ifndef ALTAIR_PWM_setup_h
#define ALTAIR_PWM_setup_h

#include <Arduino.h>

#define PWM_INITIALIZATION_VALUE 4999

void initialize_PWM_registers() {
// Set up the generic clock (GCLK7) to clock timer TCC0
 // 14.6.2.1 Initializing the Generator Control register GENCTRLn with a single 32-bit write
 // 14.7 Register Summary: 7:0 SRC; 8:15 GEN (including the DIVSEL bit); 23:16 DIV[7:0]; 31:24 DIV[15:8]
 // If GENCTRLn.DIVSEL=0 and GENCTRLn.DIV is either 0 or 1, the output clock will be undivided.
 // 14.8.3 Detailed description of every bit in the GENCTRLn register
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 48MHz clock source by divisor 1: 48MHz/1 = 48MHz
                         GCLK_GENCTRL_IDC |          // Improve Duty Cycle IDC sets the clock source duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
                         //GCLK_GENCTRL_SRC_DPLL1;     // Select 100MHz DPLL clock source
                         //GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL7);               // Wait for synchronization  

// 14.6.3 Set up Peripheral Clock by selecting the previously set up GCLK7 and mapping it to TCC0
  GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC0 peripheral channel (see Table 14-9 on p. 157 of SAMD data sheet for mapping 25 -> TCC0)
                          GCLK_PCHCTRL_GEN_GCLK7;    // Connect generic clock 7 GCLK7

  // Enable the peripheral multiplexer on pin D33
  PORT->Group[g_APinDescription[33].ulPort].PINCFG[g_APinDescription[33].ulPin].bit.PMUXEN = 1;
  
  // Set the D33 (PORT_PA20) peripheral multiplexer to peripheral (even port number -- 20 is even) E(6): TCC0, Channel 0. 
  // (The analogous operation for odd port numbers would be PMUXO -- for more details, see p. 825 of SAMD data sheet.  
  //  Note that E(6) of PMUXE, per p. 825 of the data sheet, is peripheral function G: for that, see row containing "PB12"
  //  near middle of p. 33 of the SAMD data sheet: you'll find TCC0/WO[0] (i.e. TCC0, Channel 0) there.)
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
  TCC0->PER.reg = 119999;                            // Set-up the PER (period) register 50Hz PWM (6MHz/119999 ~= 6MHz/120000 = 50Hz)
  while (TCC0->SYNCBUSY.bit.PER);                    // Wait for synchronization

  // Define Duty cycle with comparison to the PWM Period
  // Initialization value for the motors when they power up. This defines their operating range. Rotation starts at ~300 more than this initialization value
  TCC0->CC[0].reg = PWM_INITIALIZATION_VALUE;              // Set-up the CC (counter compare), channel 0 register
  while (TCC0->SYNCBUSY.bit.CC0);                    // Wait for synchronization
  TCC0->CC[1].reg = PWM_INITIALIZATION_VALUE;              // Set-up the CC (counter compare), channel 1 register
  while (TCC0->SYNCBUSY.bit.CC1);                    // Wait for synchronization
  TCC0->CC[2].reg = PWM_INITIALIZATION_VALUE;              // Set-up the CC (counter compare), channel 2 register
  while (TCC0->SYNCBUSY.bit.CC2);                    // Wait for synchronization
  TCC0->CC[3].reg = PWM_INITIALIZATION_VALUE;              // Set-up the CC (counter compare), channel 0 register
  while (TCC0->SYNCBUSY.bit.CC3);                    // Wait for synchronization

  // Enable
  TCC0->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC0
  while (TCC0->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization

  Serial.println("PWM control pins initialized!");

}

void PWM_motor_control(int PWM_pinChoice){
    int PWM_control_value = 0;
    int PWM_value = 0;

  switch (PWM_pinChoice) {
    case 1:
      // All Motors
      Serial.println("Set control value for all motors from 0 to 10: ");
      while (Serial.available() == 0) {}
      PWM_control_value = Serial.parseInt();
      if(PWM_control_value == 0){
        PWM_value = PWM_INITIALIZATION_VALUE;
      }else{
        PWM_value = PWM_INITIALIZATION_VALUE + 200 + 100*PWM_control_value;
      }
      for(int i = 0; i < 4; i++) {TCC0->CCBUF[i].reg = PWM_value;}
      break;
   
    
    case 30:
      // Pin D30 = POR_PA23 = TCC0, Channel 3
      Serial.println("Set control value from 0 to 10: ");
      while (Serial.available() == 0) {}
      PWM_control_value = Serial.parseInt();
      if(PWM_control_value == 0){
        PWM_value = PWM_INITIALIZATION_VALUE;
      }else{
        PWM_value = PWM_INITIALIZATION_VALUE + 200 + 100*PWM_control_value;
      }
      TCC0->CCBUF[3].reg = PWM_value;
      break;

    case 31:
      // Pin D31 = POR_PA22 = TCC0, Channel 2
      Serial.println("Set control value from 0 to 10: ");
      while (Serial.available() == 0) {}
      PWM_control_value = Serial.parseInt();
      if(PWM_control_value == 0){
        PWM_value = PWM_INITIALIZATION_VALUE;
      }else{
        PWM_value = PWM_INITIALIZATION_VALUE + 200 + 100*PWM_control_value;
      }
      TCC0->CCBUF[2].reg = PWM_value;
      break;

    case 32:
      // Pin D32 = POR_PA21 = TCC0, Channel 1
      Serial.println("Set control value from 0 to 10: ");
      while (Serial.available() == 0) {}
      PWM_control_value = Serial.parseInt();
      if(PWM_control_value == 0){
        PWM_value = PWM_INITIALIZATION_VALUE;
      }else{
        PWM_value = PWM_INITIALIZATION_VALUE + 200 + 100*PWM_control_value;
      }
      TCC0->CCBUF[1].reg = PWM_value;
      break;

    case 33:
      // Pin D33 = POR_PA20 = TCC0, Channel 0
      Serial.println("Set control value from 0 to 10: ");
      while (Serial.available() == 0) {}
      PWM_control_value = Serial.parseInt();
      if(PWM_control_value == 0){
        PWM_value = PWM_INITIALIZATION_VALUE;
      }else{
        PWM_value = PWM_INITIALIZATION_VALUE + 200 + 100*PWM_control_value;
      }
      TCC0->CCBUF[0].reg = PWM_value;
      break;

    default:
      Serial.println("Please choose a valid selection. Motors are connected to Pins D30, D31, D32 and D33");
      break;
  }
  Serial.print("PWM on Pin "); Serial.print(PWM_pinChoice);
  Serial.print(" set to "); Serial.println(PWM_control_value);
}
#endif      //  ifndef ALTAIR_PWM_setup_h