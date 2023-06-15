/**************************************************************************/
/*!
    @file     ALTAIR_TempSensor.cpp
    @author   Christopher Vogt (christophervogt@uvic.ca)
    @license  GPL

    
    Christopher Vogt (christophervogt@uvic.ca)     began on 12 June 2023

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_TempSensor.h"
#include "Arduino.h"

/**************************************************************************/
/*!
 @brief  Default constructor.
*/
/**************************************************************************/
ALTAIR_TempSensor::ALTAIR_TempSensor() {
    
}


/**************************************************************************/
/*!
 @brief  Set analog input pin of Arduino Micro for Temp Sensor
*/
/**************************************************************************/
void ALTAIR_TempSensor::initializeTempSensor(int analogInputPin) {
    _analogInputPin = analogInputPin;
}


/**************************************************************************/
/*!
 @brief  Store latest analog Inputs (as given by TEMP_AVERAGING_WINDOW_SIZE)
*/
/**************************************************************************/
void ALTAIR_TempSensor::storeAnalogTemp() {
    
    for (int i = 0; i < TEMP_AVEREGING_WINDOW_SIZE; i++) {
        _analogTemp = analogRead(_analogInputPin);  
        _tempWindow[i] = _analogTemp;
    }
    _tempWindowSum = 0;
    for (int i = 0; i < TEMP_AVEREGING_WINDOW_SIZE; i++) {
        _tempWindowSum = _tempWindowSum + _tempWindow[i];
    }
    }


/**************************************************************************/
/*!
 @brief  Calculate Temperature in °C 
*/
/**************************************************************************/
void ALTAIR_TempSensor::calculateTemp() {

    // Sensor itself gives output between 0 to 5 V. This scale is directly tied to the Kelvin Scale
    // TempKelvin = VoltogeIn_mV / 10
    // Resolution therefore is 0.49K    
    _tempK = (static_cast<float>(_tempWindowSum)/(1023*TEMP_AVEREGING_WINDOW_SIZE)) * 500;
    _temp = _tempK-273.15;
}

/**************************************************************************/
/*!
 @brief  Pack Temperature for I2C transfer. (-128 to +128 °C)
        With unsigned short better to transfer in Kelvin.
        Scaled by 2, because measurement resolution is ~0.5K
*/
/**************************************************************************/
unsigned short ALTAIR_TempSensor::packTemp(float theTemp) {
    unsigned short thePackedTemp;
    thePackedTemp = static_cast<unsigned short>(theTemp*2);
    return thePackedTemp;
}

byte ALTAIR_TempSensor::packTemp_byte(float theTemp) {
    byte thePackedTemp;
    if (theTemp >= 0. && theTemp < 127.) {
        thePackedTemp = theTemp;
    } else if (theTemp > -127. && theTemp < 0.) {
        thePackedTemp = theTemp + 256.;
    } else if (theTemp >= 127.) {
        thePackedTemp = 127;
    } else {
        thePackedTemp = 128;
    }
    return thePackedTemp;
}
