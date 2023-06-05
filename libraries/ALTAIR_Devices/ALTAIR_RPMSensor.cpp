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
    RPMSensor.setTypeAnalog();
    RPMSensor.setSensorPins((const uint8_t*) { analog_input_pin }, (const uint8_t) 1);
    _analog_input_pin = int(analog_input_pin);
    //Serial.println("RPM Sensor initialized");
}


/**************************************************************************/
/*!
 @brief  Store latest analog Input of Arduino Micro into analog_rpm 
*/
/**************************************************************************/
void ALTAIR_RPMSensor::store_analog_RPM() {
    
    //Serial.println("Start analog storage");
    //RPMSensor.read((uint16_t*) analog_rpm);       //Grand Central needs this to compile
    //RPMSensor.read(&analog_rpm);
    //Serial.println("read");
    //Serial.println((int)analog_rpm);
    //Serial.println("---");
    

    analog_rpm = analogRead(_analog_input_pin);
    
    
    // Update the RPM window sum by subtracting the oldest value and adding the new value
    RPM_windowSum = RPM_windowSum - RPM_window[_window_Index] + analog_rpm;
    //Serial.println((int)RPM_windowSum);

    RPM_window[_window_Index] = analog_rpm;
    _window_Index = (_window_Index + 1) % RPM_AVEREGING_WINDOW_SIZE;
    //Serial.print("stored "); Serial.println(RPM_window[0]); 
}

/**************************************************************************/
/*!
 @brief  Edge detection in RPM Window
*/
/**************************************************************************/
void ALTAIR_RPMSensor::Edge_detection() {
    //Serial.println("Start rising Edge detection");
    
    RPM_windowAverage = static_cast<float>(RPM_windowSum) / RPM_AVEREGING_WINDOW_SIZE;
    if (analog_rpm > RPM_windowAverage + RPM_EDGE_DETECTION_THRESHOLD) {
        if(!_isRisingEdge) {
            RPM_risingEdge_counter++;
            _isRisingEdge = 1;
            //Serial.println("Rising Edge detected");
        }
        _isFallingEdge = 0;
    }
    else if (analog_rpm < RPM_windowAverage - RPM_EDGE_DETECTION_THRESHOLD) {
        if(!_isFallingEdge) {
            RPM_fallingEdge_counter++;
            _isFallingEdge = 1;
            //Serial.println("Falling Edge detected");
        }
        _isRisingEdge = 0;
    }
}



/**************************************************************************/
/*!
 @brief  Rising Edge detection in RPM Window
*/
/**************************************************************************/
void ALTAIR_RPMSensor::risingEdge_detection() {
    //Serial.println("Start rising Edge detection");
    RPM_windowAverage = RPM_windowSum / RPM_AVEREGING_WINDOW_SIZE;
    if (analog_rpm > RPM_windowAverage + RPM_EDGE_DETECTION_THRESHOLD) {
        RPM_risingEdge_counter++;
    }
}

/**************************************************************************/
/*!
 @brief  Falling Edge detection in RPM Window
*/
/**************************************************************************/
void ALTAIR_RPMSensor::fallingEdge_detection() {
    //Serial.println("Start falling Edge detection");
    RPM_windowAverage = RPM_windowSum / RPM_AVEREGING_WINDOW_SIZE;
    if (analog_rpm < RPM_windowAverage - RPM_EDGE_DETECTION_THRESHOLD) {
        RPM_fallingEdge_counter++;
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
    _rpm_rising   = static_cast<float>(RPM_risingEdge_counter * 60000) / (RPM_NUMBER_OF_TAPES * rpm_measurement_millis);
    _rpm_falling  = static_cast<float>(RPM_fallingEdge_counter * 60000) / (RPM_NUMBER_OF_TAPES * rpm_measurement_millis);
    if (abs(RPM_risingEdge_counter - RPM_fallingEdge_counter) <= 2) {
        rpm = (_rpm_rising + _rpm_falling) / 2;
    }
}
