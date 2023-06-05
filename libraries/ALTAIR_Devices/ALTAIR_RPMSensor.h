/**************************************************************************/
/*!
    @file     ALTAIR_RPMSensor.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class of each RPM sensor for each of the four ALTAIR 
    propulsion motors.  These optical sensors are Pololu QTR-1A reflectance 
    sensors (https://www.pololu.com/product/2458) that are connected to the 
    Arduino Micro to establish a running-averaged RPM measurement.  Four 
    (and only four) objects of this ALTAIR_RPMSensor class should end up 
    instantiated; they will each be instantiated by their respective 
    ALTAIR_MotorAndESC object upon the singleton instantiation of the
    ALTAIR_PropulsionSystem object.

    Justin Albert  jalbert@uvic.ca     began on 2 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_RPMSensor_h
#define ALTAIR_RPMSensor_h

#include "Arduino.h"
#include "QTRSensors.h"

#define RPM_AVEREGING_WINDOW_SIZE        3
#define RPM_EDGE_DETECTION_THRESHOLD     100
#define RPM_NUMBER_OF_TAPES              2

class ALTAIR_RPMSensor {
  public:
      ALTAIR_RPMSensor();
      //ALTAIR_RPMSensor(int test);
      //ALTAIR_RPMSensor(const uint8_t analog_input_pin);

      float                 rpm = 0.;
      float                 _rpm_rising = 0.;
      float                 _rpm_falling = 0.;
      int                   _isRisingEdge = 0;
      int                   _isFallingEdge = 0;
      int                   _window_Index = 0;
      uint16_t              analog_rpm = 0                                 ;
      int                   _analog_input_pin = 0;

      const uint8_t SensorCount = 1;
      uint16_t sensorValues[1];

      int                   RPM_risingEdge_counter = 0;
      int                   RPM_fallingEdge_counter = 0;
      uint16_t              RPM_window[RPM_AVEREGING_WINDOW_SIZE] = {0};
      uint16_t              RPM_windowSum = 0;
      float                 RPM_windowAverage = 0.;
      QTRSensors RPMSensor;
  //float                   rpm(              )     { return _rpm       ;   }
  //void                    setRPM( float rpm )     {        _rpm = rpm ;   }
    void                    initialize_QTRsensor(const uint8_t* analog_input_pin);
    void                    store_analog_RPM(   );
    void                    Edge_detection();
    void                    risingEdge_detection();
    void                    fallingEdge_detection();
    void                    calculate_RPM(int rpm_measurement_millis);
    //void                    provideRPM_I2C(     );
    //void                    retrieveRPM_I2C(    ) ;

    

  
      
    

};
#endif    //   ifndef ALTAIR_RPMSensor_h
