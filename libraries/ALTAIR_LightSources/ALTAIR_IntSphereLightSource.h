/**************************************************************************/
/*!
    @file     ALTAIR_IntSphereLightSource.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR integrating sphere-based light 
    source, including its four monochromatic light sources (three laser
    diode modules, at 440 nm, 635 nm, and 670 nm, and one 
    frequency-doubled Nd:YAG laser module at 532 nm).  It inherits from 
    the ALTAIR_LightSource base class.

    Justin Albert  jalbert@uvic.ca     began on 29 Aug. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_IntSphereLightSource_h
#define ALTAIR_IntSphereLightSource_h

#include "ALTAIR_LightSource.h"

#define  DEFAULT_GREEN532NMLASERPIN   40
#define  DEFAULT_RED670NMLASERPIN     41
#define  DEFAULT_RED635NMLASERPIN     42
#define  DEFAULT_BLUE440NMLASERPIN    43

class ALTAIR_IntSphereLightSource : public ALTAIR_LightSource {
  public:
    virtual void    initialize()                                                             ;
    virtual void    resetLights()                                                            ;   // set lights to their previous state (before flash start)
    virtual void    setLightsPrimaryRadio()                                                  ;   // test flash pattern when primary  radio is transmitting
    virtual void    setLightsBackupRadio()                                                   ;   // test flash pattern when a backup radio is transmitting

    virtual void    turnOffLasers()                                                          ;
    virtual void    flashBlueLaser()                                                         ;
    virtual void    flashRed635nmLaser()                                                     ;

    void            turnOnBlueLaser()                                                        ;
    void            turnOffBlueLaser()                                                       ;
    void            turnOnGreenLaser()                                                       ;
    void            turnOffGreenLaser()                                                      ;
    void            turnOnRed635nmLaser()                                                    ;
    void            turnOffRed635nmLaser()                                                   ;
    void            turnOnRed670nmLaser()                                                    ;
    void            turnOffRed670nmLaser()                                                   ;

    uint8_t         getStatusNibble()                                                        ;

    ALTAIR_IntSphereLightSource(const char blue440nmLaserPin                               , 
                                const char green532nmLaserPin = DEFAULT_GREEN532NMLASERPIN ,
                                const char red635nmLaserPin   = DEFAULT_RED635NMLASERPIN   ,
                                const char red670nmLaserPin   = DEFAULT_RED670NMLASERPIN   ) ;
    ALTAIR_IntSphereLightSource()                                                            ;  // Default constructor => all default arguments

  protected:

  private:
    char  _green532nmLaserPin;
    char  _red670nmLaserPin;
    char  _red635nmLaserPin;
    char  _blue440nmLaserPin;

    bool  _green532nmLaserState;
    bool  _red670nmLaserState;
    bool  _red635nmLaserState;
    bool  _blue440nmLaserState;
};
#endif    //   ifndef ALTAIR_IntSphereLightSource_h

