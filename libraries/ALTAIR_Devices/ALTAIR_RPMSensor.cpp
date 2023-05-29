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

#include "C:\Users\Chris\source\repos\ProjectALTAIR\ALTAIROnboardArduinoSoftware\libraries\ALTAIR_Devices\ALTAIR_RPMSensor.h"
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
 @brief  Test constructor.
*/
/**************************************************************************/
ALTAIR_RPMSensor::ALTAIR_RPMSensor(int i) {
    Serial.println("hello");
}

/**************************************************************************/
/*!
 @brief  Constructor equipping Sensor with the correct analog input pin
*/
/**************************************************************************/
ALTAIR_RPMSensor::ALTAIR_RPMSensor(uint8_t analog_input_pin) {
    RPMSensor.setTypeAnalog();
    RPMSensor.setSensorPins((uint8_t[]) { analog_input_pin }, 1);
}

/**************************************************************************/
/*!
 @brief  Store latest analog Input of Arduino Micro into analog_rpm 
*/
/**************************************************************************/
void ALTAIR_RPMSensor::store_analog_RPM() {
    RPMSensor.read(analog_rpm);
    // Update the RPM window sum by subtracting the oldest value and adding the new value
    RPM_windowSum = RPM_windowSum - RPM_window[RPM_AVEREGING_WINDOW_SIZE - 1] + analog_rpm;
    for (int i = 1; i < RPM_AVEREGING_WINDOW_SIZE; i++) {
        RPM_window[RPM_AVEREGING_WINDOW_SIZE - i] = RPM_window[RPM_AVEREGING_WINDOW_SIZE - (i + 1)];
    }
    RPM_window[0] = analog_rpm;
}

/**************************************************************************/
/*!
 @brief  Rising Edge detection in RPM Window
*/
/**************************************************************************/
void ALTAIR_RPMSensor::risingEdge_detection() {
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
    RPM_windowAverage = RPM_windowSum / RPM_AVEREGING_WINDOW_SIZE;
    if (analog_rpm < RPM_windowAverage + RPM_EDGE_DETECTION_THRESHOLD) {
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
    float rpm_rising  = (RPM_risingEdge_counter / RPM_NUMBER_OF_TAPES) * (60 / (1000*rpm_measurement_millis));
    float rpm_falling = (RPM_fallingEdge_counter / RPM_NUMBER_OF_TAPES) * (60 / (1000*rpm_measurement_millis));
    if (abs(rpm_rising - rpm_falling) < 5) {
        rpm = (rpm_rising + rpm_falling) / 2;
    }
}
