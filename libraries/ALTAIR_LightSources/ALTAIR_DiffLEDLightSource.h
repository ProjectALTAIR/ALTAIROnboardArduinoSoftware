/**************************************************************************/
/*!
    @file     ALTAIR_DiffLEDLightSource.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the ALTAIR diffusive LED light source, 
    including its four LED light sources (blue, green, yellow, and red).
    It inherits from the ALTAIR_LightSource base class.

    Justin Albert  jalbert@uvic.ca     began on 29 Aug. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_DiffLEDLightSource_h
#define ALTAIR_DiffLEDLightSource_h

#include "ALTAIR_LightSource.h"

#define  DEFAULT_YELLOWLEDSPIN   36
#define  DEFAULT_REDLEDSPIN      37
#define  DEFAULT_BLUELEDSPIN     38
#define  DEFAULT_GREENLEDSPIN    39

class ALTAIR_DiffLEDLightSource : public ALTAIR_LightSource {
  public:
    virtual void    initialize()                                                ;
    virtual void    resetLights()                                               ;
    virtual void    setLightsPrimaryRadio()                                     ;   // test flash pattern when primary  radio is transmitting
    virtual void    setLightsBackupRadio()                                      ;   // test flash pattern when a backup radio is transmitting

    virtual void    turnOffLEDs()                                               ;
    virtual void    flashYellowLEDs()                                           ;
    virtual void    flashRedLEDs()                                              ;
    virtual void    flashBlueLEDs()                                             ;

    void            turnOnBlueLEDs()                                            ;
    void            turnOffBlueLEDs()                                           ;
    void            turnOnGreenLEDs()                                           ;
    void            turnOffGreenLEDs()                                          ;
    void            turnOnYellowLEDs()                                          ;
    void            turnOffYellowLEDs()                                         ;
    void            turnOnRedLEDs()                                             ;
    void            turnOffRedLEDs()                                            ;

    uint8_t         getStatusNibble()                                           ;

    ALTAIR_DiffLEDLightSource(const char blueLEDsPin                            , 
                              const char greenLEDsPin  = DEFAULT_GREENLEDSPIN   ,
                              const char yellowLEDsPin = DEFAULT_YELLOWLEDSPIN  ,
                              const char redLEDsPin    = DEFAULT_REDLEDSPIN   ) ;
    ALTAIR_DiffLEDLightSource()                                                 ;  // Default constructor => all default arguments

  protected:

  private:
    char           _yellowLEDsPin                                               ;
    char           _redLEDsPin                                                  ;
    char           _blueLEDsPin                                                 ;
    char           _greenLEDsPin                                                ;

    bool           _yellowLEDsState                                             ;
    bool           _redLEDsState                                                ;
    bool           _blueLEDsState                                               ;
    bool           _greenLEDsState                                              ;
};
#endif    //   ifndef ALTAIR_DiffLEDLightSource_h

