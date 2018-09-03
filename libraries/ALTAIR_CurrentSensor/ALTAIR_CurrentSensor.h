/**************************************************************************/
/*!
    @file     ALTAIR_CurrentSensor.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class of each current sensor for each of the four ALTAIR 
    propulsion motor/ESC pairs.  These non-contact, open loop miniature 
    current sensors are Honeywell model CSLT6B100 sensors which go 
    around a power line leading to each ESC, and produce a voltage 
    proportional to the current going through that line.  With no current,
    their output voltage is (V_in)/2 (where V_in = 5 V), and their output
    voltage increases or decreases by approximately 16 mV per ampere that
    flows through the power line they surround.

    Justin Albert  jalbert@uvic.ca     began on 2 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_CurrentSensor_h
#define ALTAIR_CurrentSensor_h

#include "Arduino.h"

class ALTAIR_CurrentSensor {
  public:

    ALTAIR_CurrentSensor() : _current(         0.       ) {                             }

    float                     current(                  ) { return _current           ; }
    void                      setCurrent( float current ) {        _current = current ; }

  private:
    float                    _current                                                 ;    // in Amps

};
#endif    //   ifndef ALTAIR_CurrentSensor_h
