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
void ALTAIR_RPMSensor::initialize_QTRsensor(const uint8_t* analog_input_pin) {
    _RPMSensor.setTypeAnalog();
    _RPMSensor.setSensorPins((const uint8_t*) { analog_input_pin }, (const uint8_t) 1);
    _analog_input_pin = int(analog_input_pin);
    //Serial.println("RPM Sensor initialized");
}


/**************************************************************************/
/*!
 @brief  Store latest analog Input of Arduino Micro into analog_rpm 
*/
/**************************************************************************/
void ALTAIR_RPMSensor::store_analog_RPM() {
    
    //RPMSensor.read((uint16_t*) analog_rpm);       //Grand Central needs this to compile
    //RPMSensor.read(&analog_rpm);

    _analog_rpm = analogRead(_analog_input_pin);  
    
    // Update the RPM window sum by subtracting the oldest value and adding the new value
    _RPM_windowSum = _RPM_windowSum - _RPM_window[_window_Index] + _analog_rpm;

    _RPM_window[_window_Index] = _analog_rpm;
    _window_Index = (_window_Index + 1) % RPM_AVEREGING_WINDOW_SIZE;
}

/**************************************************************************/
/*!
 @brief  Edge detection in RPM Window
*/
/**************************************************************************/
void ALTAIR_RPMSensor::Edge_detection() {
    //Serial.println("Start rising Edge detection");
    
    _RPM_windowAverage = static_cast<float>(_RPM_windowSum) / RPM_AVEREGING_WINDOW_SIZE;
    if (_analog_rpm > _RPM_windowAverage + RPM_EDGE_DETECTION_THRESHOLD) {
        if(!_isRisingEdge) {
            _risingEdge_counter++;
            _isRisingEdge = 1;
            //Serial.println("Rising Edge detected");
        }
        _isFallingEdge = 0;
    }
    else if (_analog_rpm < _RPM_windowAverage - RPM_EDGE_DETECTION_THRESHOLD) {
        if(!_isFallingEdge) {
            _fallingEdge_counter++;
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
void ALTAIR_RPMSensor::calculate_RPM(int rpm_measurement_millis) {
    //Serial.println("Start RPM calculation");
    _rpm_rising   = static_cast<float>(_risingEdge_counter * 60000) / (RPM_NUMBER_OF_TAPES * rpm_measurement_millis);
    _rpm_falling  = static_cast<float>(_fallingEdge_counter * 60000) / (RPM_NUMBER_OF_TAPES * rpm_measurement_millis);
    if (abs(_risingEdge_counter - _fallingEdge_counter) <= 2) {
        _rpm = (_rpm_rising + _rpm_falling) / 2;
    }
}

/**************************************************************************/
/*!
 @brief  Pack RPM for I2C transfer. (Resolution limited to 60RPM, maxes at ~7200)
*/
/**************************************************************************/
byte ALTAIR_RPMSensor::packRPS(float theRPM) {
    float theRPS = theRPM/60.;
    _packRPS;
    if (theRPS >= 0. && theRPS < 127.) {
        _packRPS = theRPS;
    } else if (theRPS >= -128. && theRPS < 0.) {
        _packRPS = theRPS + 256.;
    } else if (theRPS >= 127.) {
        _packRPS = 127;
    } else {
        _packRPS = 128;
    }
    return _packRPS;
}
