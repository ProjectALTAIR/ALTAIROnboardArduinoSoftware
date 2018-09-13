/**************************************************************************/
/*!
    @file     ALTAIR_GPSSensors.h
    @author   Justin Albert (jalbert@uvic.ca)
    @license  GPL

    This is the class for the two ALTAIR GPS receivers (the NEO-M8N on the 
    mast, and the DFRobot G6 located in a small plastic housing directly 
    atop the payload).

    Justin Albert  jalbert@uvic.ca     began on 6 Sep. 2018

    @section  HISTORY

    v1.0  - First version
*/
/**************************************************************************/

#ifndef ALTAIR_GPSSensors_h
#define ALTAIR_GPSSensors_h

#include "Arduino.h"
#include "ALTAIR_NEOM8N.h"
#include "ALTAIR_DFRobotG6.h"

class ALTAIR_GPSSensors {
  public:

    ALTAIR_GPSSensors(                        )                     ;

    ALTAIR_NEOM8N*          neom8n(           )  { return &_neom8n  ; }
    ALTAIR_DFRobotG6*       dfrobot(          )  { return &_dfrobot ; }

    ALTAIR_GPSSensor*       primary(          )  { return  _primary ; }              
    ALTAIR_GPSSensor*       backup(           )  { return  _backup  ; } 

    void                    initialize(       )                     ;
    void                    switchToOtherGPS( )                     ;

  private:
    ALTAIR_NEOM8N          _neom8n                                  ;
    ALTAIR_DFRobotG6       _dfrobot                                 ;

    ALTAIR_GPSSensor*      _primary                                 ; 
    ALTAIR_GPSSensor*      _backup                                  ; 
};
#endif    //   ifndef ALTAIR_GPSSensors_h
