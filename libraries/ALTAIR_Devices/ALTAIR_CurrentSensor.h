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

#define CURRENT_SENSOR_COUNT              4
#define CURRENT_AVEREGING_WINDOW_SIZE     100

class ALTAIR_CurrentSensor {
  public:

    ALTAIR_CurrentSensor();
    //ALTAIR_CurrentSensor() :  _current(         0.    )  {                       }

    float                   current(               )  { return _current        ; }
    int                     analogCurrent(         )  { return _analogCurrent  ; }
    int*                     currentWindow(         )  { return _currentWindow;}
    float                   currentWindowSum(  )  { return _currentWindowSum;}
    int                     analogInputPin(     )  { return _analogInputPin;}    
    //void                    setCurrent( float current )  {        _current = current ; }

    void                    initializeCurrentSensor(int analogInputPin);
    void                    storeAnalogCurrent();
    void                    calculateCurrent(      );
    unsigned short          packCurrent_short(float theCurrent);
    byte                    packCurrent(float theCurrent);

  private:
    int                    _analogInputPin;
    int                    _currentWindow[CURRENT_AVEREGING_WINDOW_SIZE];
    float                  _currentWindowSum;

    int                    _analogCurrent                                   ;   // analog input
    float                  _current                                         ;   // in mA   // in Amperes

};
#endif    //   ifndef ALTAIR_CurrentSensor_h
