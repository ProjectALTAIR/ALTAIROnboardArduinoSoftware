/**************************************************************************/
/*!
    @file     ALTAIR_CurrentSensor.cpp
    @author   Christopher Vogt (christophervogt@uvic.ca)
    @license  GPL

    
    Christopher Vogt (christophervogt@uvic.ca)     began on 12 June 2023

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_CurrentSensor.h"
#include "Arduino.h"

/**************************************************************************/
/*!
 @brief  Default constructor.
*/
/**************************************************************************/
ALTAIR_CurrentSensor::ALTAIR_CurrentSensor() {
    
}


/**************************************************************************/
/*!
 @brief  Set analog input pin of Arduino Micro for Temp Sensor
*/
/**************************************************************************/
void ALTAIR_CurrentSensor::initializeCurrentSensor(int analogInputPin) {
    _analogInputPin = analogInputPin;
}


/**************************************************************************/
/*!
 @brief  Store latest analog Inputs (as given by TEMP_AVERAGING_WINDOW_SIZE)
*/
/**************************************************************************/
void ALTAIR_CurrentSensor::storeAnalogCurrent() {
    
    for (int i = 0; i < CURRENT_AVEREGING_WINDOW_SIZE; i++) {
        _analogCurrent = analogRead(_analogInputPin);  
        _currentWindow[i] = _analogCurrent;
    }
    _currentWindowSum = 0;
    for (int i = 0; i < CURRENT_AVEREGING_WINDOW_SIZE; i++) {
        _currentWindowSum = _currentWindowSum + _currentWindow[i];
    }
    }


/**************************************************************************/
/*!
 @brief  Calculate Current in mA
        Vout = (Load-Resistor (100kOhm) / 10kOhm) * I (Amps)
        Range is 0 to 500mA
        Resolution therefore is ~0.5mA
*/
/**************************************************************************/
void ALTAIR_CurrentSensor::calculateCurrent() {
   
    _current = ((_currentWindowSum  /CURRENT_AVEREGING_WINDOW_SIZE) * (static_cast<float>(5000) / 10230));
}

/**************************************************************************/
/*!
 @brief  Pack Current for I2C transfer. Scaled down:
            //Range is 0 to 500mA --> 0 to 124 with 4mA resolution
        as unsigned short no scaling needed, directly mA transferred
*/
/**************************************************************************/
unsigned short ALTAIR_CurrentSensor::packCurrent(float theCurrent) {
    unsigned short thePackedCurrent = theCurrent * 1000;       // whole numbers 0-65535 mapped onto 0.-65.535mA (65535mikroA)
    return thePackedCurrent;
}

byte ALTAIR_CurrentSensor::packCurrent_byte(float theCurrent) {
    float scaledCurrent = theCurrent * 10;      // whole numbers 0-255 mapped onto by 0-25.5mA
    byte thePackedCurrent;
    if (scaledCurrent >= 0. && scaledCurrent < 255.) {
        thePackedCurrent = scaledCurrent;
    } else if (scaledCurrent >= 255.) {
        thePackedCurrent = 255;
    } else {
        thePackedCurrent = 0;
    }
    return thePackedCurrent;
}
