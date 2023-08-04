/**************************************************************************/
/*!
    @file     ALTAIR_RPMSensor.cpp
    @author   Christopher Vogt (christophervogt@uvic.ca)
    @license  GPL

    
    Christopher Vogt (christophervogt@uvic.ca)     began on 24 May 2023

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

#include "ALTAIR_RPMSensor.h"

//#include "C:\Users\Chris\OneDrive - bwedu\Dokumente\Arduino\ALTAIR\libraries\ALTAIR_Devices\ALTAIR_RPMSensor.h"
#include "Arduino.h"

/**************************************************************************/
/*!
 @brief  Default constructor.
*/
/**************************************************************************/
ALTAIR_RPMSensor::ALTAIR_RPMSensor() {
    
}

/**************************************************************************/
/*!
 @brief  Constructor equipping Sensor with the correct analog input pin
*/
/**************************************************************************/
/* ALTAIR_RPMSensor::ALTAIR_RPMSensor(const uint8_t analog_input_pin) {
    RPMSensor.setTypeAnalog();
    RPMSensor.setSensorPins((const uint8_t*) { analog_input_pin }, (const uint8_t) 1);
} */

/**************************************************************************/
/*!
 @brief  Initialize QTRSensor and set analog input pin of Arduino Micro 
*/
/**************************************************************************/
void ALTAIR_RPMSensor::initializeQTRsensor(const uint8_t* analog_input_pin) {
    _RPMSensor.setTypeAnalog();
    _RPMSensor.setSensorPins((const uint8_t*) { analog_input_pin }, (const uint8_t) 1);
    _analogInputPin = int(analog_input_pin);
    //Serial.println("RPM Sensor initialized");
}


/**************************************************************************/
/*!
 @brief  Store latest analog Input of Arduino Micro into analog_rpm 
*/
/**************************************************************************/
void ALTAIR_RPMSensor::storeAnalogRPM() {
    
    //RPMSensor.read((uint16_t*) analog_rpm);       //Grand Central needs this to compile
    //RPMSensor.read(&analog_rpm);

    _analogRPM = analogRead(_analogInputPin);  
    
    // Update the RPM window sum by subtracting the oldest value and adding the new value
    _RPMWindowSum = _RPMWindowSum - _RPMWindow[_windowIndex] + _analogRPM;

    _RPMWindow[_windowIndex] = _analogRPM;
    _windowIndex = (_windowIndex + 1) % RPM_AVEREGING_WINDOW_SIZE;
}

/**************************************************************************/
/*!
 @brief  Edge detection in RPM Window
*/
/**************************************************************************/
void ALTAIR_RPMSensor::EdgeDetection() {
    //Serial.println("Start rising Edge detection");
    
    _RPMWindowAverage = static_cast<float>(_RPMWindowSum) / RPM_AVEREGING_WINDOW_SIZE;
    if (_analogRPM > _RPMWindowAverage + RPM_EDGE_DETECTION_THRESHOLD) {
        if(!_isRisingEdge) {
            _risingEdgeCounter++;
            _isRisingEdge = 1;
            //Serial.println("Rising Edge detected");
        }
        _isFallingEdge = 0;
    }
    else if (_analogRPM < _RPMWindowAverage - RPM_EDGE_DETECTION_THRESHOLD) {
        if(!_isFallingEdge) {
            _fallingEdgeCounter++;
            _isFallingEdge = 1;
            //Serial.println("Falling Edge detected");
        }
        _isRisingEdge = 0;
    }
}

/**************************************************************************/
/*!
 @brief  Calculate RPM given the count of rising and falling Edges and the 
        time period of the sensing.
*/
/**************************************************************************/
void ALTAIR_RPMSensor::calculateRPM(int rpm_measurement_millis) {
    //Serial.println("Start RPM calculation");
    _rpmRising   = static_cast<float>(_risingEdgeCounter * 60000) / (RPM_NUMBER_OF_TAPES * rpm_measurement_millis);
    _rpmFalling  = static_cast<float>(_fallingEdgeCounter * 60000) / (RPM_NUMBER_OF_TAPES * rpm_measurement_millis);
    if (abs(_risingEdgeCounter - _fallingEdgeCounter) <= 2) {
        _rpm = (_rpmRising + _rpmFalling) / 2;
    }
}

/**************************************************************************/
/*!
 @brief  Pack RPM for I2C transfer. (Resolution limited to 60RPM, maxes at ~7200)
*/
/**************************************************************************/
unsigned short ALTAIR_RPMSensor::packRPM_short(float theRPM) {
    unsigned short thePackedRPS = theRPM;
    return thePackedRPS;
}

byte ALTAIR_RPMSensor::packRPS(float theRPM) {
    float _theRPS = theRPM/60.;
    byte thePackedRPS;
    if (_theRPS >= 0. && _theRPS < 255.0) {
        thePackedRPS = _theRPS;
    } else if (_theRPS >= 255.0) {
        thePackedRPS = 255;
    } else {
        thePackedRPS = 0;
    }
    return thePackedRPS;
}
