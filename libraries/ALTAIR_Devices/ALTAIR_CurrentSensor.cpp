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
        Vout    = (Load-Resistor (100kOhm) * Shunt-Resistor (0.001 Ohm) / 1kOhm) * I (Amps) 
                = 0.1 (Ohm) * I (Amps)
                = analog_input/1023 * 5
        Range is 0 to 50A
        Resolution therefore is ~50mA (per sensor reading)
*/
/**************************************************************************/
void ALTAIR_CurrentSensor::calculateCurrent() {
   
    _current = ((_currentWindowSum  /CURRENT_AVEREGING_WINDOW_SIZE) * (static_cast<float>(50000) / 1023));
}

/**************************************************************************/
/*!
 @brief  Pack Current for I2C transfer. Scaled down:
         
        
*/
/**************************************************************************/
unsigned short ALTAIR_CurrentSensor::packCurrent_short(float theCurrent) {
    unsigned short thePackedCurrent = theCurrent * 10;       // whole numbers 0-65535 mapped onto 0-6.5535A (6553.5milliA)
    return thePackedCurrent;
}

byte ALTAIR_CurrentSensor::packCurrent(float theCurrent) {
    float scaledCurrent = theCurrent / 10;      // whole numbers 0-255 mapped onto by 0-2.55A
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
