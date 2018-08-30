/**************************************************************************/
/*!
    @file     ALTAIR_LightSource.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the base class for ALTAIR light sources, including the
    integrating sphere-based light source containing the laser diode
    light sources, the diffusive light source containing the LED light
    sources, as well as any other sources (e.g. microwave polarimetric) 
    that may be added in the future.
    Note that the light source-specific classes (e.g. 
    IntSphereLightSource, DiffLEDLightSource, etc.) derive from this 
    class.  Essentially all functionality (i.e. anything that isn't
    entirely specific to a given light source) is abstracted to and
    available within this interface.  Note that when necessary, inputs
    default to those for the integrating sphere-based light source. 
    Justin Albert  jalbert@uvic.ca     began on 29 Aug. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_LightSource_h
#define ALTAIR_LightSource_h

#include "Arduino.h"

class ALTAIR_LightSource {
  public:
    virtual void    initialize()            = 0                                 ;
    virtual void    setLightsNormal()       = 0                                 ;
    virtual void    setLightsPrimaryRadio() = 0                                 ;   // test flash pattern when primary  radio is transmitting
    virtual void    setLightsBackupRadio()  = 0                                 ;   // test flash pattern when a backup radio is transmitting
    virtual bool    isInitialized()         { return _isInitialized             ; }
    virtual bool    isOn()                  { return _isOn                      ; }

  protected:
    ALTAIR_LightSource()                                                        ;

    virtual void    turnOn()                { if (_isInitialized) _isOn = true  ; }
    virtual void    turnOff()               { if (_isInitialized) _isOn = false ; }
    virtual void    setInitialized()        { _isInitialized            = true  ; }

  private:
    bool            _isInitialized                                              ;
    bool            _isOn                                                       ;
};
#endif    //   ifndef ALTAIR_LightSource_h

