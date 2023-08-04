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

#define RPM_SENSOR_COUNT                 4
#define RPM_AVEREGING_WINDOW_SIZE        3
#define RPM_EDGE_DETECTION_THRESHOLD     100
#define RPM_NUMBER_OF_TAPES              2

class ALTAIR_RPMSensor {
  public:
    ALTAIR_RPMSensor();
      
    float                   rpm()                       { return _rpm       ;           }
    uint16_t                analogRPM()                { return _analogRPM;           }
    int                     risingEdgeCounter()        { return _risingEdgeCounter;   }   // Getter for Debugging
    void                    resetRisingEdgeCounter()   { _risingEdgeCounter  = 0;     }
    int                     fallingEdgeCounter()       { return _fallingEdgeCounter;  }   // Getter for Debuging
    void                    resetFallingEdgeCounter()  { _fallingEdgeCounter = 0;     }
    int                     analogInputPin()  	    { return _analogInputPin;     } // Not needed, just for debugging
    QTRSensors*             RPMSensor()                 { return &_RPMSensor;           }

    void                    initializeQTRsensor(const uint8_t* analog_input_pin);
    void                    storeAnalogRPM(   );
    void                    EdgeDetection();
    void                    calculateRPM(int rpm_measurement_millis);
    unsigned short          packRPM_short(float theRPM);
    byte                    packRPS(float theRPM);

    private:
        int                 _analogInputPin = 0;

        uint16_t            _analogRPM = 0                                 ;
        float               _rpm = 0.;
        float               _rpmRising = 0.;
        float               _rpmFalling = 0.;
        
        int                 _windowIndex = 0;
        uint16_t            _RPMWindow[RPM_AVEREGING_WINDOW_SIZE] = {0};
        uint16_t            _RPMWindowSum = 0;
        float               _RPMWindowAverage = 0.;

        int                 _risingEdgeCounter = 0;
        int                 _fallingEdgeCounter = 0;
        int                 _isRisingEdge = 0;
        int                 _isFallingEdge = 0;

        QTRSensors          _RPMSensor;


};
#endif    //   ifndef ALTAIR_RPMSensor_h
