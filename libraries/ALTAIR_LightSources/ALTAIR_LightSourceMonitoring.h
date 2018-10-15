/**************************************************************************/
/*!
    @file     ALTAIR_LightSourceMonitoring.h
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

#ifndef   ALTAIR_LightSourceMonitoring_h
#define   ALTAIR_LightSourceMonitoring_h

#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1015.h>    // is the proper header for the Adafruit ADS 1115 ADCs (as well)

#define   DEFAULT_ADS1115ADC_I2CADDRESS               0x48
#define   DEFAULT_ADS1115ADC2_I2CADDRESS              0x49
#define   DEFAULT_ADS1115ADC3_I2CADDRESS              0x4A
#define   DEFAULT_ADS1115ADC4_I2CADDRESS              0x4B
#define   INTSPHERE_PD1_ADC_CHANNEL                      0             //  located on both ads1115ADC1 and ads1115ADC2
#define   INTSPHERE_PD2_ADC_CHANNEL                      1             //  located on both ads1115ADC1 and ads1115ADC2
#define   INTSPHERE_PD3_ADC_CHANNEL                      2             //  located on ads1115ADC2
#define   INTSPHERE_SHX_RSSI_ADC_CHANNEL                 3             //  located on ads1115ADC2


class ALTAIR_LightSourceMonitoring {
  public:
    virtual void         initialize(  )                           ;

    Adafruit_ADS1115*    ads1115ADC1( )    { return &_ads1115ADC1 ; }
    Adafruit_ADS1115*    ads1115ADC2( )    { return &_ads1115ADC2 ; }
    Adafruit_ADS1115*    ads1115ADC3( )    { return &_ads1115ADC3 ; }
    Adafruit_ADS1115*    ads1115ADC4( )    { return &_ads1115ADC4 ; }

    ALTAIR_LightSourceMonitoring(     )                           ;

  protected:

  private:

    Adafruit_ADS1115    _ads1115ADC1                              ;
    Adafruit_ADS1115    _ads1115ADC2                              ;
    Adafruit_ADS1115    _ads1115ADC3                              ;
    Adafruit_ADS1115    _ads1115ADC4                              ;
};
#endif    //   ifndef ALTAIR_LightSourceMonitoring_h

