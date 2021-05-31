/**************************************************************************/
/*!
    @file     ALTAIR_LightSourceMonitoring.cpp
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This class contains methods for initializing and controlling all
    ALTAIR light source monitoring electronics.  This includes the 
    photodiode amplifier boards and the ADC boards (and their associated 
    environmental monitoring).

    Justin Albert  jalbert@uvic.ca     began on 8 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#include "ALTAIR_LightSourceMonitoring.h"

/**************************************************************************/
/*!
 @brief  Default constructor.
*/
/**************************************************************************/
ALTAIR_LightSourceMonitoring::ALTAIR_LightSourceMonitoring(  ) :
   _ads1115ADC1(                                             ) ,
   _ads1115ADC2(                                             ) ,
   _ads1115ADC3(                                             ) ,
   _ads1115ADC4(                                             )
{

}

/**************************************************************************/
/*!
 @brief  Initialize the three ADS1115 boards (within the setup routine).
*/
/**************************************************************************/
void ALTAIR_LightSourceMonitoring::initialize(               )
{
    Serial.println(F("Initializing the four ADS1115 4-channel ADC breakout boards ..."));
   _ads1115ADC1.begin(        DEFAULT_ADS1115ADC_I2CADDRESS  );
   _ads1115ADC2.begin(        DEFAULT_ADS1115ADC2_I2CADDRESS );
   _ads1115ADC3.begin(        DEFAULT_ADS1115ADC3_I2CADDRESS );
   _ads1115ADC4.begin(        DEFAULT_ADS1115ADC4_I2CADDRESS );
}

