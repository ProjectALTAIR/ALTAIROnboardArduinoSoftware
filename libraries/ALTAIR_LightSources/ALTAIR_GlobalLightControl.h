/**************************************************************************/
/*!
    @file     ALTAIR_GlobalLightControl.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This class contains methods for initializing and controlling all 
    ALTAIR light sources and light source monitoring electronics.
    The former presently includes the integrating sphere light source
    (with its 4 laser sources), and the diffusive LED light source (with
    its 4 colours of LEDs).  The latter includes the photodiode amplifier
    boards and the ADC boards (and their associated environmental 
    monitoring).

    This class should be instantiated as a singleton.

    Justin Albert  jalbert@uvic.ca     began on 7 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef   ALTAIR_GlobalLightControl_h
#define   ALTAIR_GlobalLightControl_h

#include "Arduino.h"

#include "ALTAIR_IntSphereLightSource.h"
#include "ALTAIR_DiffLEDLightSource.h"    
#include "ALTAIR_LightSourceMonitoring.h"   // includes the amplifiers and ADC boards

class ALTAIR_GlobalLightControl {
  public:

    ALTAIR_GlobalLightControl(                                              )                            ;

    bool                            initializeAllLightSources(              )                            ;

    void                            performCommand(       byte  commandByte )                            ;

    ALTAIR_IntSphereLightSource*    intSphereSource(                        ) { return &_intSphereSource ; }
    ALTAIR_DiffLEDLightSource*      diffLEDSource(                          ) { return &_diffLEDSource   ; } 
    ALTAIR_LightSourceMonitoring*   lightSourceMon(                         ) { return &_lightSourceMon  ; } // includes the amplifiers and ADC boards

    uint8_t                         getLightStatusByte(                     )                            ;

  protected:

  private:
    ALTAIR_IntSphereLightSource    _intSphereSource                                                      ;
    ALTAIR_DiffLEDLightSource      _diffLEDSource                                                        ;
    ALTAIR_LightSourceMonitoring   _lightSourceMon                                                       ;

};
#endif    //   ifndef ALTAIR_GlobalLightControl_h
