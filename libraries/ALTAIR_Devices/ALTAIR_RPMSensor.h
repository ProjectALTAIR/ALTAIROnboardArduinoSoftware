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

class ALTAIR_RPMSensor {
  public:

    ALTAIR_RPMSensor() :    _rpm(         0.   )     {                     }

    float                    rpm(              )     { return _rpm       ; }
    void                     setRPM( float rpm )     {        _rpm = rpm ; }

  private:
    float                   _rpm                                         ;

};
#endif    //   ifndef ALTAIR_RPMSensor_h
