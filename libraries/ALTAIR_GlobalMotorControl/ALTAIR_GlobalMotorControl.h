/**************************************************************************/
/*!
    @file     ALTAIR_GlobalMotorControl.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This class contains static methods for initializing and controlling 
    _all_ ALTAIR motors (including both the 4 propulsion motors as well 
    as the 3 servo motors) all at once.

    This class should be instantiated as a singleton.

    Justin Albert  jalbert@uvic.ca     began on 2 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_GlobalMotorControl_h
#define ALTAIR_GlobalMotorControl_h

#include "Arduino.h"

#include "ALTAIR_PropulsionSystem.h"
#include "ALTAIR_BleedSystem.h"
#include "ALTAIR_CutdownSystem.h"

class ALTAIR_GlobalMotorControl {
  public:

    ALTAIR_GlobalMotorControl(                                                             ) ;

    bool  initializeAllMotors(                                                             ) ;

    void  performCommand(             byte                     commandByte                 ) ;

    ALTAIR_PropulsionSystem& propSystem( )        { return    _propSystem                    ; }
    ALTAIR_BleedSystem&     bleedSystem( )        { return   _bleedSystem                    ; }
    ALTAIR_CutdownSystem& cutdownSystem( )        { return _cutdownSystem                    ; }

    bool               shutDownAllProps( )        { return    _propSystem.shutDownAllProps() ; }   // Return power setting of all props to 0.  Returns true if successful.

  protected:
    void  initializeServoControlRegisters(                                                 ) ;


  private:
    ALTAIR_PropulsionSystem  _propSystem                                                     ;
    ALTAIR_BleedSystem       _bleedSystem                                                    ;
    ALTAIR_CutdownSystem     _cutdownSystem                                                  ;

};
#endif    //   ifndef ALTAIR_GlobalMotorControl_h
